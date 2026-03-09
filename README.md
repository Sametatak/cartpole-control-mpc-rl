# Cart-Pole Control: MPC and Reinforcement Learning in ROS 2

This repository contains a comparative implementation of **Model Predictive Control (MPC)** and **Reinforcement Learning (RL)** strategies to stabilize an inverted pendulum on a cart.  
The project is built on **ROS 2 Humble** and simulated in **Gazebo**.

---

# 📌 Project Overview

The goal of this project is to keep a dynamically unstable **cart-pole system upright** while actively rejecting external disturbances.

The repository includes:

- **Linear MPC (LMPC):** A highly optimized controller providing precise stability.
- **Nonlinear MPC (NMPC):** An experimental non-linear approach.
- **Reinforcement Learning (PPO):** A custom **Sim-to-Sim-to-Real pipeline** using a **10-robot Gazebo environment** to train a robust recovery policy.
- **Teleop Node:** Manual keyboard control of the cart.

---

# ROS 2 Architecture

The control nodes interact with the Gazebo simulation via the following topics:

**Subscribe**
```
/joint_states
```
Receives:
- Cart position
- Cart velocity
- Pole angle
- Pole angular velocity

**Publish**
```
/cart_force
```
Applies linear force to the cart to maintain balance.

---

# ⚙️ Prerequisites & Dependencies

Before cloning the repository ensure the following are installed:

**Operating System**

```
Ubuntu 22.04
```

**ROS Version**

```
ROS 2 Humble 
```

**Simulator**

```
Gazebo
```

**Python**

```
Python 3.10+
```

---

# Python Libraries

Install the required Python packages:

```bash
pip install numpy scipy gymnasium stable-baselines3 tensorboard
pip install osqp    # Required for Linear MPC
pip install casadi  # Required for Nonlinear MPC
```

---

# 🚀 Installation & Build

### 1. Source ROS 2 environment

```bash
source /opt/ros/humble/setup.bash
```

### 2. Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clone Repository

Replace **YOUR_USERNAME** with your GitHub username.

```bash
git clone https://github.com/YOUR_USERNAME/cartpole-control-mpc-rl.git
```

### 4. Build Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 5. Source Workspace

```bash
source install/setup.bash
```

---

# 🎮 Usage Instructions

---

# Part 1: Model Predictive Control (MPC)

### 1. Launch Simulation World

Start the **single-robot Gazebo environment**:

```bash
ros2 launch robot_launch launch_simulation.launch.py
```

---

### 2. Run Controllers

Open a **new terminal for each controller**.

#### Linear MPC (Recommended)

Stabilizes the pole and periodically injects disturbances to test robustness.

```bash
ros2 run my_mpc_planner python_mpc_node
```

---

#### Nonlinear MPC (Experimental)

Note: This controller is included for research purposes and currently performs slower due to solver latency.

```bash
ros2 run my_mpc_planner python_nmpc_node
```

---

#### Manual Control (Teleop)

Apply force manually using keyboard control.

```bash
ros2 run my_mpc_planner teleop.py
```

---

# Part 2: Reinforcement Learning (PPO)

Training directly in Gazebo is extremely slow, so we use a **Fast-Sim Pretraining Pipeline**.

---

# Method A: Optimized Pipeline (Recommended)

### Step 1 — High-Speed Pretraining

Train the agent using a fast mathematical simulation (**6000+ FPS**) without launching Gazebo.

```bash
cd ~/ros2_ws/src/my_mpc_planner/my_mpc_planner
python3 fast_simulation.py
```

---

### Step 2 — Launch Multi-Robot Gazebo Environment

```bash
ros2 launch robot_launch multiple_robot.launch.py
```

---

### Step 3 — Gazebo Refinement & Testing

Load the pre-trained model and continue training inside Gazebo.

```bash
ros2 run my_mpc_planner python_rl_node2
```

---

# Method B: Direct Real-Time Training (Extremely Slow)

Train the RL agent directly inside Gazebo.

### Terminal 1

```bash
ros2 launch robot_launch multiple_robot.launch.py
```

### Terminal 2

```bash
ros2 run my_mpc_planner python_rl_node
```

---

# 📊 Project Features

- Linear MPC with **OSQP solver**
- Experimental Nonlinear MPC using **CasADi**
- Reinforcement Learning using **Stable-Baselines3 PPO**
- **10 parallel robots** for accelerated training
- Disturbance rejection testing


---
