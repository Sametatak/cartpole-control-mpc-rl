import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
from std_srvs.srv import Empty

import numpy as np
import scipy.signal
import cvxpy as cp
import math
import random

class PythonMpcNode(Node):
    def __init__(self):
        super().__init__('python_mpc_node')
        
        self.publisher_ = self.create_publisher(Wrench, '/cart_force', 10)
        self.subscription_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.reset_client = self.create_client(Empty, '/reset_simulation')
        
        # --- FİZİKSEL PARAMETRELER (Task 1 PDF uyumlu) ---
        self.M = 1.0     # Araba kütlesi [cite: 10]
        self.m = 0.2     # Sarkaç kütlesi [cite: 11]
        self.l = 0.5     # Kütle merkezine uzaklık [cite: 12]
        self.g = 9.81    # Yerçekimi [cite: 13]
        self.Ts = 0.03   # 50 Hz refleks hızı
        self.bc = 0.1    # Araba sönümleme
        self.bp = 0.01
        # Dinamik Matrisler (A, B)
        self.Ac = np.array([
            [0, 1, 0, 0],
            [0, -self.bc/self.M, self.m * self.g / self.M, 0],
            [0, 0, 0, 1],
            [0, -self.bc/(self.l * self.M), (self.M + self.m) * self.g / (self.l * self.M), -self.bp/(self.m * self.l**2)]
        ])
        self.Bc = np.array([[0], [1/self.M], [0], [1/(self.l * self.M)]])
        self.Cc = np.eye(4)
        self.Dc = np.zeros((4, 1))
        
        sys_d = scipy.signal.cont2discrete((self.Ac, self.Bc, self.Cc, self.Dc), self.Ts, method='zoh')
        self.Ad = sys_d[0]
        self.Bd = sys_d[1]
        
        # --- MPC KURULUMU ---
        self.N = 50  
        self.Q = np.diag([25.0, 1.0, 500.0, 10.0]) 
        self.R = np.array([[0.06]]) 
        
        self.u_min = -40.0
        self.u_max = 40.0
        self.x_min = -10.0
        self.x_max = 10.0
        self.x_ref = np.array([0.0, 0.0, 0.0, 0.0]) # Hedef: 0 konumu ve 0 açısı 
        
        self.current_x = 0.0
        self.current_x_dot = 0.0
        self.current_theta = 0.0
        self.current_theta_dot = 0.0

        # CVXPY Hazırlığı
        self.x_init = cp.Parameter(4) 
        self.x_var = cp.Variable((4, self.N + 1))
        self.u_var = cp.Variable((1, self.N))
        
        cost = 0
        constraints = [self.x_var[:, 0] == self.x_init]
        for k in range(self.N):
            cost += cp.quad_form(self.x_var[:, k] - self.x_ref, self.Q) + cp.quad_form(self.u_var[:, k], self.R)
            constraints += [self.x_var[:, k+1] == self.Ad @ self.x_var[:, k] + self.Bd @ self.u_var[:, k]]
            constraints += [self.u_min <= self.u_var[:, k], self.u_var[:, k] <= self.u_max]
            constraints += [self.x_min <= self.x_var[0, k], self.x_var[0, k] <= self.x_max] 
            
        cost += cp.quad_form(self.x_var[:, self.N] - self.x_ref, self.Q)
        self.prob = cp.Problem(cp.Minimize(cost), constraints)
        
        # --- TEST DURUM MAKİNESİ ---
        self.mode = 'START_EPISODE'
        self.stable_ticks = 0
        self.disturb_ticks = 0
        self.disturb_force = 0.0
        
        self.timer = self.create_timer(self.Ts, self.timer_callback)

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "cart_joint":
                self.current_x = msg.position[i]
                self.current_x_dot = msg.velocity[i]
            elif name == "pole_joint":
                self.current_theta = -msg.position[i]     
                self.current_theta_dot = -msg.velocity[i] 

    def timer_callback(self):
        u_opt = 0.0

        if self.mode == 'START_EPISODE':
            # Başlangıç açısı kuralı [cite: 29]
            self.target_angle = random.uniform(0.174, 0.349) 
            self.nudge_direction = random.choice([-1, 1])
            self.nudge_ticks = 0
            self.mode = 'NUDGING'
            self.stable_ticks = 0
            print(f"\n[DENGELİ MOD] Yeni Bölüm Başladı. Hedef: {math.degrees(self.target_angle):.1f}°")

        elif self.mode == 'NUDGING':
            u_opt = self.nudge_direction * 15.0
            self.nudge_ticks += 1
            if self.nudge_ticks > 2:
                self.mode = 'WAIT_FOR_DROP'
                u_opt = 0.0

        elif self.mode == 'WAIT_FOR_DROP':
            if abs(self.current_theta) >= self.target_angle:
                self.mode = 'MPC_CONTROL'

        elif self.mode == 'MPC_CONTROL':
            if abs(self.current_theta) > 1.22 or abs(self.current_x) > 9.8:
                self.reset_system()
                return

            # MPC Çözümü
            u_opt = self.solve_mpc()

            # --- STABİLİTE TAKİBİ ---
            # Eğer açı < 1.5 derece ve konum merkeze yakınsa sayaç işlesin
            if abs(self.current_theta) < 0.026 and abs(self.current_x) < 0.2:
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0 # Sarsıntı olursa sayaç başa döner

            # 10 Saniye Geçti mi? (10 sn / Ts)
            if self.stable_ticks >= (2.0 / self.Ts):
                self.mode = 'DISTURBANCE'
                self.disturb_ticks = 0
                self.disturb_force = random.uniform(6, 13.0)*random.choice([1,-1]) # Sert bir darbe
                print(f"\n[!] DARBE UYGULANIYOR: {self.disturb_force} N")

        elif self.mode == 'DISTURBANCE':
            u_opt = self.disturb_force
            self.disturb_ticks += 1
            # Darbe 0.1 saniye (5 tick) sürsün
            if self.disturb_ticks > 5:
                print("\n[V] Darbe bitti, MPC kurtarmaya çalışıyor!")
                self.mode = 'MPC_CONTROL'
                self.stable_ticks = 0 # Tekrar 10 sn sayması için sıfırla

        elif self.mode == 'WAIT_RESET':
            if self.future_reset.done():
                self.mode = 'START_EPISODE'
            u_opt = 0.0

        # Mesajı Gönder
        wrench_msg = Wrench()
        wrench_msg.force.x = float(u_opt)
        self.publisher_.publish(wrench_msg)
        
        if self.mode in ['MPC_CONTROL', 'DISTURBANCE']:
            stable_time = self.stable_ticks * self.Ts
            print(f"\rStabil: {stable_time:>4.1f}s | T:{math.degrees(self.current_theta):>5.1f}° | U:{u_opt:>6.2f}N ", end="")

    def solve_mpc(self):
        self.x_init.value = np.array([self.current_x, self.current_x_dot, self.current_theta, self.current_theta_dot])
        try:
            self.prob.solve(solver=cp.OSQP, warm_start=True) 
            if self.prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                return self.u_var[:, 0].value[0] 
        except:
            pass
        return 0.0

    def reset_system(self):
        print(f"\n[X] ÇÖKÜŞ! Resetleniyor...")
        if self.reset_client.wait_for_service(timeout_sec=0.5):
            req = Empty.Request()
            self.future_reset = self.reset_client.call_async(req)
            self.mode = 'WAIT_RESET'

def main(args=None):
    rclpy.init(args=args)
    node = PythonMpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
