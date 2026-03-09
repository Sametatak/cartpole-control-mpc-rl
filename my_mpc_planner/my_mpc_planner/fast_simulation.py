import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math
import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
from stable_baselines3.common.monitor import Monitor

class FastMathCartPoleEnv(gym.Env):
    def __init__(self):
        super(FastMathCartPoleEnv, self).__init__()
        
        # --- PDF'TEKİ FİZİKSEL DEĞERLER ---
        self.gravity = 9.81
        self.masscart = 1.0
        self.masspole = 0.2  # PDF'te 0.2 kg istenmiş
        self.total_mass = (self.masspole + self.masscart)
        self.length = 0.5    # PDF'te 0.5 m istenmiş
        self.polemass_length = (self.masspole * self.length)
        self.force_mag = 25.0
        self.tau = 0.05      # Gazebo ile senkronize 20Hz
        
        self.action_space = spaces.Box(low=-self.force_mag, high=self.force_mag, shape=(1,), dtype=np.float32)
        
        high = np.array([4.8, np.inf, math.pi, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        
        self.state = None
        self.current_step = 0
        self.max_steps = 400

    def step(self, action):
        self.current_step += 1
        x, x_dot, theta, theta_dot = self.state
        force = np.clip(action[0], -self.force_mag, self.force_mag)
        
        costheta = math.cos(theta)
        sintheta = math.sin(theta)
        
        temp = (force + self.polemass_length * theta_dot**2 * sintheta) / self.total_mass
        thetaacc = (self.gravity * sintheta - costheta * temp) / (self.length * (4.0/3.0 - self.masspole * costheta**2 / self.total_mass))
        xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass
        
        # Euler Integrasyonu
        x = x + self.tau * x_dot
        x_dot = x_dot + self.tau * xacc
        theta = theta + self.tau * theta_dot
        theta_dot = theta_dot + self.tau * thetaacc
        
        self.state = (x, x_dot, theta, theta_dot)
        
        # Sınır kontrolleri
        terminated = bool(x < -4.0 or x > 4.0 or theta < -1.085 or theta > 1.085)
        truncated = bool(self.current_step >= self.max_steps)
        
        # Ödül Fonksiyonu
        if terminated:
            reward = -50.0
        else:
            reward = 1.0 # Hayatta kalma ödülü
            reward -= (theta ** 2) * 10.0
            reward -= (x ** 2) * 0.1

        return np.array(self.state, dtype=np.float32), float(reward), terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # PDF ŞARTI: theta_0 ~ U(-20, 20) derece
        low_angle = math.radians(-20)
        high_angle = math.radians(20)
        
        self.state = np.array([
            np.random.uniform(-0.1, 0.1),
            np.random.uniform(-0.05, 0.05),
            np.random.uniform(low_angle, high_angle),
            np.random.uniform(-0.05, 0.05)
        ], dtype=np.float32)
        
        self.current_step = 0
        return self.state, {}

def make_env():
    # Rollout kaydı için Monitor şart!
    env = FastMathCartPoleEnv()
    return Monitor(env)

def main():
    print("\n" + "="*50)
    print("[+] ROLLOUT DESTEKLİ HIZLI EĞİTİM BAŞLIYOR")
    print("="*50)
    
    log_dir = "./fast_math_logs_v5/"
    os.makedirs(log_dir, exist_ok=True)
    
    # 10 paralel ortamı Monitor ile oluşturup VecMonitor ile birleştiriyoruz
    env = DummyVecEnv([make_env for _ in range(10)])
    env = VecMonitor(env) # Bu satır rollout/ep_rew_mean verisini sağlar
    
    model = PPO("MlpPolicy", env, verbose=1, 
                n_steps=1024, 
                batch_size=256,
                learning_rate=0.0003, # Daha stabil öğrenme için
                tensorboard_log=log_dir)

    model.learn(total_timesteps=1000000, tb_log_name="PPO_Math_Train")
    
    model.save("ppo_cartpole_math_brain")
    print(f"\n[+] EĞİTİM BİTTİ! Loglar şurada: {log_dir}")

if __name__ == '__main__':
    main()
