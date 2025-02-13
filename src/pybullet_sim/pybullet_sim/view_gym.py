import gymnasium as gym
from pybullet_sim.crowd_gym import CrowdAvoidanceEnv


def main():
    """Create and visualize the CrowdAvoidanceEnv."""
    env = CrowdAvoidanceEnv(use_gui=True)
    env.reset()
    
    while True:
        action = env.action_space.sample()  # Sample a random action
        observation, reward, done, _, _ = env.step(action)
        
        
        if done:
            env.reset()

if __name__ == "__main__":
    main()