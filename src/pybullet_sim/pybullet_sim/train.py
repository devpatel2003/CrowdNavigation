import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.logger import configure
from crowd_gym import CrowdAvoidanceEnv
import time  # Needed for rendering
import pybullet as p

# Create training environment with GUI
env = CrowdAvoidanceEnv()


# Enable TensorBoard logging
log_dir = "./sac_logs"
logger = configure(log_dir, ["stdout", "tensorboard"])

# Set up evaluation callback
eval_env = CrowdAvoidanceEnv()
eval_callback = EvalCallback(eval_env, best_model_save_path="./logs/",
                             log_path="./logs/", eval_freq=5000)

# ? Define SAC model with proper parameters
model = SAC(
    "MlpPolicy",
    env,
    learning_rate=1e-4,       # Learning rate (default: 3e-4)
    batch_size=64,           # Batch size for training
    buffer_size=200000,      # Replay buffer size
    tau=0.01,                # Target network update rate
    gamma=0.98,               # Discount factor
    train_freq=(50, "step"),# Train after every episode
    gradient_steps=2,         # Number of gradient updates per step
    policy_kwargs={"net_arch": [512, 512]},  # Neural network architecture
    verbose=1,                # Logging level (1 = info, 0 = silent)
    tensorboard_log="./sac_logs/"
)

#model = SAC.load("sac_crowd_avoidance_align_50k", env=env)
# ? Attach logger to model
model.set_logger(logger)

print("Model Created. Training begins...")

# ? Train model with proper logging
TIMESTEPS = 150000

model.learn(total_timesteps=TIMESTEPS, callback=eval_callback, progress_bar=True, reset_num_timesteps=False)

# ? Save final trained model
model.save("sac_crowd_avoidance_align_150k")
print("Training Complete! Model Saved.")