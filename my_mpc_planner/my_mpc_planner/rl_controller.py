import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
from std_srvs.srv import Empty

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
import threading
import math
import random
from stable_baselines3 import PPO

# ==========================================
# 1. ÇOKLU ROBOT ROS 2 BRIDGE (Dünya Resetlemeli)
# ==========================================
class GazeboMultiBridgeNode(Node):
    def __init__(self, num_robots=10):
        super().__init__('gazebo_multi_rl_bridge')
        self.num_robots = num_robots
        self.publishers_ = []
        self.states = np.zeros((num_robots, 4), dtype=np.float32)
        self.received_flags = [False] * num_robots
        
        self.reset_client = self.create_client(Empty, '/reset_simulation')

        for i in range(num_robots):
            ns = f'/robot_{i}'
            pub = self.create_publisher(Wrench, f'{ns}/cart_force', 10)
            self.publishers_.append(pub)
            self.create_subscription(
                JointState, 
                f'{ns}/joint_states', 
                lambda msg, idx=i: self.joint_state_callback(msg, idx), 
                10)

    def joint_state_callback(self, msg, idx):
        for i, name in enumerate(msg.name):
            if "cart_joint" in name:
                self.states[idx][0] = msg.position[i]
                self.states[idx][1] = msg.velocity[i]
            elif "pole_joint" in name:
                self.states[idx][2] = msg.position[i]
                self.states[idx][3] = msg.velocity[i]
        self.received_flags[idx] = True

    def send_forces(self, forces):
        for i in range(self.num_robots):
            msg = Wrench()
            msg.force.x = float(forces[i])
            self.publishers_[i].publish(msg)

    def reset_world_sim(self):
        """Tüm Gazebo dünyasını sıfırlar"""
        if not self.reset_client.service_is_ready():
            self.get_logger().error("/reset_world servisi bulunamadi!")
            return

        # Kuvvetleri sıfırla ki resetlenince uçmasınlar
        zero_forces = [0.0] * self.num_robots
        self.send_forces(zero_forces)

        # Dünyayı sıfırla
        req = Empty.Request()
        self.reset_client.call_async(req)
        
        self.received_flags = [False] * self.num_robots

# ==========================================
# 2. GYMNASIUM ENVIRONMENT (Dayanıklılık Korumalı)
# ==========================================
class MultiGazeboCartPoleEnv(gym.Env):
    def __init__(self):
        super(MultiGazeboCartPoleEnv, self).__init__()
        self.num_robots = 10
        self.node = GazeboMultiBridgeNode(num_robots=self.num_robots)
        
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        self.action_space = spaces.Box(low=-25.0, high=25.0, shape=(self.num_robots,), dtype=np.float32)
        high = np.array([2.5, np.inf, math.pi/2, np.inf] * self.num_robots, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        # Eğitim Süresi ve Yaşam Durumu Değişkenleri
        self.max_steps = 400 # 20 saniye
        self.current_step = 0
        self.dead_robots = [False] * self.num_robots
        
        # Puan ve Stabilite Takibi
        self.episode_total_rewards = np.zeros(self.num_robots, dtype=np.float32)
        self.stable_steps = np.zeros(self.num_robots, dtype=int) # SARSINTI SAYACI

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        self.node.reset_world_sim()
        time.sleep(0.5)
        
        # Başlangıç itkisi
        init_pushes = [random.uniform(-10.0, 10.0) for _ in range(self.num_robots)]
        self.node.send_forces(init_pushes)
        
        while not all(self.node.received_flags):
            time.sleep(0.05)
            
        # Değişkenleri Sıfırla
        self.current_step = 0
        self.dead_robots = [False] * self.num_robots 
        self.episode_total_rewards = np.zeros(self.num_robots, dtype=np.float32)
        self.stable_steps = np.zeros(self.num_robots, dtype=int) # Sayacı sıfırla
        
        return self.node.states.flatten(), {}

    def step(self, action):
        self.current_step += 1
        
        safe_actions = []
        obs = self.node.states.copy()
        
        # 1. SARSINTI (DISTURBANCE) VE KUVVET HESAPLAMA
        for i in range(self.num_robots):
            if self.dead_robots[i]:
                safe_actions.append(0.0) # Yerde yatan robota kuvvet uygulama
            else:
                x, x_dot, theta, theta_dot = obs[i]
                final_force = float(action[i])
                
                # Stabilite Kontrolü: Çubuk +- 0.1 radyan (yaklaşık 5.7 derece) içindeyse stabil say
                if abs(theta) < 0.1:
                    self.stable_steps[i] += 1
                else:
                    self.stable_steps[i] = 0 # Dengesi bozulduysa sayacı sıfırla
                
                # EĞER 5 SANİYE (100 ADIM) STABİL KALDIYSA HAFİF TOKAT AT!
                if self.stable_steps[i] >= 100:
                    disturbance = random.uniform(-10.0, 10.0) # <--- GÜNCELLENEN KISIM: -10N ile 10N arası
                    final_force += disturbance # Ajanın kendi kuvvetine dış darbeyi ekle
                    self.stable_steps[i] = 0 # Tokadı yedi, sayacı hemen sıfırla
                
                safe_actions.append(final_force)
                
        self.node.send_forces(safe_actions)
        time.sleep(0.05)
        
        # Yeni durumları oku
        obs = self.node.states.copy()
        rewards = []
        
        # 2. ÖDÜL VE ÖLÜM KONTROLÜ
        for i in range(self.num_robots):
            # Eğer robot önceden düştüyse (yerde yatıyorsa)
            if self.dead_robots[i]:
                rewards.append(-2.0) # Yerde yattığı her adım için ceza ver
                continue

            x, x_dot, theta, theta_dot = obs[i]
            
            # Robot yeni mi düştü?
            if abs(theta) > 1.085 or abs(x) > 4.0:
                rewards.append(-50.0) # Düştüğü an yediği tokat
                self.dead_robots[i] = True # Robotu ölü işaretle
                self.stable_steps[i] = 0 # Düşenin sayacı da sıfırlanır
            else:
                # Başarıyla ayakta duruyor
                r = 1.0 - (abs(theta)/0.785)**2
                rewards.append(r)
        
        # O adımın ödüllerini toplam listesine ekle
        for i in range(self.num_robots):
            self.episode_total_rewards[i] += rewards[i]
            
        step_reward = sum(rewards) / self.num_robots
        mean_total_reward = np.mean(self.episode_total_rewards)
        
        # 3. BİTİŞ ŞARTLARI
        terminated = all(self.dead_robots)
        truncated = bool(self.current_step >= self.max_steps)
        
        alive_count = self.num_robots - sum(self.dead_robots)
        
        # Ekrana Bilgi Bas
        print(f"\r[RL] Adım:{self.current_step:>3}/{self.max_steps} | Yaşayan:{alive_count:>2} | TOPLAM Puan:{mean_total_reward:>7.1f} | R0 Açı:{math.degrees(obs[0][2]):>5.1f}°  ", end="")
        if terminated or truncated:
            print(f"\n[BÖLÜM BİTTİ] Sebep: {'Herkes Düştü' if terminated else 'Süre Doldu (BAŞARI)'} | Final Ort. Puan: {mean_total_reward:.1f}")

        return obs.flatten(), step_reward, terminated, truncated, {}

# ==========================================
# 3. ANA ÇALIŞTIRICI (MAIN FUNCTION)
# ==========================================
def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
        
    env = MultiGazeboCartPoleEnv()
    
    model = PPO("MlpPolicy", env, verbose=1, 
                batch_size=512, 
                n_steps=2048,
                learning_rate=0.0003,
                tensorboard_log="./multi_rl_logs/")

    print("\n" + "="*50)
    print("[EĞİTİM BAŞLIYOR] 10 Robot Dayanıklılık Eğitimi (Hafif Sarsıntılı)!")
    print("="*50)
    
    try:
        model.learn(total_timesteps=1000000)
    except KeyboardInterrupt:
        print("\n[!] Egitim durduruldu, model kaydediliyor...")
    finally:
        model.save("ppo_cartpole_multi_10_robust")
        env.close()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()