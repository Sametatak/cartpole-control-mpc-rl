import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
from std_srvs.srv import Empty

import casadi as ca
import numpy as np
import math
import random

class PythonNmpcNode(Node):
    def __init__(self):
        super().__init__('python_nmpc_node')
        
        self.publisher_ = self.create_publisher(Wrench, '/cart_force', 10)
        self.subscription_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.reset_client = self.create_client(Empty, '/reset_simulation')
        
        # --- 1. FİZİKSEL PARAMETRELER (Task 1 PDF) ---
        self.M = 1.0     
        self.m = 0.2     
        self.l = 0.5     
        self.g = 9.81    
        self.Ts = 0.025  
        
        self.bc = 0.1    
        self.bp = 0.01   
        
        # --- 2. NMPC AYARLARI ---
        self.N = 15     
        self.Q = np.diag([10.0, 1.0, 500.0, 10.0]) 
        self.R = 0.3    # Senin altın oranın!
        
        self.u_min = -30.0
        self.u_max = 30.0
        self.x_min = -10.0
        self.x_max = 10.0
        
        self.setup_nmpc()
        
        self.current_x = 0.0; self.current_x_dot = 0.0
        self.current_theta = 0.0; self.current_theta_dot = 0.0

        self.mode = 'START_EPISODE'
        self.timer = self.create_timer(self.Ts, self.timer_callback)

    def setup_nmpc(self):
        self.opti = ca.Opti()
        
        self.X_var = self.opti.variable(4, self.N + 1)
        self.U_var = self.opti.variable(1, self.N)
        self.X_init = self.opti.parameter(4, 1)
        
        x = ca.MX.sym('x'); x_dot = ca.MX.sym('x_dot'); theta = ca.MX.sym('theta'); theta_dot = ca.MX.sym('theta_dot'); u = ca.MX.sym('u')
        
        # --- %100 LMPC SENKRONİZE FİZİK MODELİ ---
        I = (4/3) * self.m * self.l**2
        den = (self.M + self.m) * I + self.M * self.m * self.l**2 + self.m**2 * self.l**2 * ca.sin(theta)**2
        
        # DÜZELTME: LMPC'deki A matrisi ile işaretleri tam uyuşan x_ddot
        x_ddot = ( 
            (I + self.m*self.l**2) * u 
            - (I + self.m*self.l**2) * self.bc * x_dot 
            + self.m**2 * self.l**2 * self.g * ca.sin(theta) * ca.cos(theta) 
            + self.m * self.l * ca.cos(theta) * self.bp * theta_dot 
            + (I + self.m*self.l**2) * self.m * self.l * theta_dot**2 * ca.sin(theta)
        ) / den
        
        # DÜZELTME: LMPC'deki A ve B matrisi ile işaretleri tam uyuşan theta_ddot
        theta_ddot = (
            self.m * self.l * ca.cos(theta) * u
            + self.m * self.l * ca.cos(theta) * self.bc * x_dot
            + (self.M + self.m) * self.m * self.g * self.l * ca.sin(theta)
            - (self.M + self.m) * self.bp * theta_dot
            - self.m**2 * self.l**2 * ca.sin(theta) * ca.cos(theta) * theta_dot**2
        ) / den
        
        f = ca.Function('f', [ca.vertcat(x, x_dot, theta, theta_dot), u], [ca.vertcat(x_dot, x_ddot, theta_dot, theta_ddot)])
        
        X_k = ca.MX.sym('X_k', 4); U_k = ca.MX.sym('U_k', 1)
        k1 = f(X_k, U_k); k2 = f(X_k + self.Ts/2 * k1, U_k); k3 = f(X_k + self.Ts/2 * k2, U_k); k4 = f(X_k + self.Ts * k3, U_k)
        X_next = X_k + self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)
        F_RK = ca.Function('F_RK', [X_k, U_k], [X_next])

        self.opti.subject_to(self.X_var[:, 0] == self.X_init)
        cost = 0
        for k in range(self.N):
            self.opti.subject_to(self.X_var[:, k+1] == F_RK(self.X_var[:, k], self.U_var[:, k]))
            self.opti.subject_to(self.opti.bounded(self.x_min, self.X_var[0, k], self.x_max))
            self.opti.subject_to(self.opti.bounded(self.u_min, self.U_var[0, k], self.u_max))
            cost += ca.mtimes([self.X_var[:, k].T, self.Q, self.X_var[:, k]]) + self.R * self.U_var[:, k]**2
            
        cost += ca.mtimes([self.X_var[:, self.N].T, self.Q, self.X_var[:, self.N]])
        self.opti.minimize(cost)
        
        # OPTİMİZASYON: Hız ve istikrar dengesi
        opts = {
            "ipopt.print_level": 0, "print_time": 0,
            "ipopt.max_iter": 20,          
            "ipopt.tol": 1e-2,             
            "ipopt.acceptable_tol": 1e-1,  
            "ipopt.warm_start_init_point": "yes"
        }
        self.opti.solver("ipopt", opts)
        
        self.last_X = np.zeros((4, self.N + 1))
        self.last_U = np.zeros((1, self.N))

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "cart_joint":
                self.current_x = msg.position[i]; self.current_x_dot = msg.velocity[i]
            elif name == "pole_joint":
                self.current_theta = -msg.position[i]; self.current_theta_dot = -msg.velocity[i] 

    def timer_callback(self):
        u_opt = 0.0
        if self.mode == 'START_EPISODE':
            self.target_angle = random.uniform(0.174, 0.349)
            self.nudge_direction = random.choice([-1, 1]); self.nudge_ticks = 0; self.mode = 'NUDGING'
        elif self.mode == 'NUDGING':
            u_opt = self.nudge_direction * 15.0; self.nudge_ticks += 1
            if self.nudge_ticks > 2: self.mode = 'WAIT_FOR_DROP'
        elif self.mode == 'WAIT_FOR_DROP':
            if abs(self.current_theta) >= self.target_angle: self.mode = 'MPC_CONTROL'
        elif self.mode == 'MPC_CONTROL':
            if abs(self.current_theta) > 1.22 or abs(self.current_x) > 9.8:
                self.reset_system(); return
            
            self.opti.set_value(self.X_init, [self.current_x, self.current_x_dot, self.current_theta, self.current_theta_dot])
            self.opti.set_initial(self.X_var, self.last_X); self.opti.set_initial(self.U_var, self.last_U)
            
            try:
                sol = self.opti.solve()
                u_opt = sol.value(self.U_var)[0]
                self.last_X = sol.value(self.X_var); self.last_U = sol.value(self.U_var)
            except Exception:
                u_opt = 0.0
            print(f"\rX: {self.current_x:>5.2f}m | T: {math.degrees(self.current_theta):>5.1f}° | U: {u_opt:>6.2f} N   ", end="")
        elif self.mode == 'WAIT_RESET':
            if self.future_reset.done():
                self.mode = 'START_EPISODE'
                self.last_X = np.zeros((4, self.N + 1)); self.last_U = np.zeros((1, self.N))
        
        wrench_msg = Wrench(); wrench_msg.force.x = float(u_opt)
        self.publisher_.publish(wrench_msg)

    def reset_system(self):
        if self.reset_client.wait_for_service(timeout_sec=0.5):
            req = Empty.Request(); self.future_reset = self.reset_client.call_async(req); self.mode = 'WAIT_RESET'

def main(args=None):
    rclpy.init(args=args); node = PythonNmpcNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
