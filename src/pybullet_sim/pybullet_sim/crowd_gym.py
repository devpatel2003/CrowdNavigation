import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import random
import os
import ament_index_python.packages
import math

ENV_BOUNDARY = 2.0  # The robot operates within -2 to 2 in x and y
MIN_DISTANCE = ENV_BOUNDARY * 0.3  # The robot and goal must be at least x% of the boundary apart
EP_LENGTH = 2000  # Max steps per episode

class MovingObstacle:
    def __init__(self, position, robot):
        """Initialize a moving obstacle."""
        self.body = p.loadURDF("cube.urdf", position, globalScaling=0.2)
        self.robot = robot

        # Enable collision with the robot only
        p.setCollisionFilterPair(self.body, self.robot, -1, -1, enableCollision=True)

        # Random movement direction (small steps)
        self.direction = np.array([
            random.uniform(-0.02, 0.02), 
            random.uniform(-0.02, 0.02), 
            0
        ])

    def move(self):
        """Move the obstacle and reverse direction at boundaries."""
        pos, _ = p.getBasePositionAndOrientation(self.body)
        new_pos = np.array(pos) + self.direction

        # Reverse direction if it reaches boundary
        if abs(new_pos[0]) > ENV_BOUNDARY or abs(new_pos[1]) > ENV_BOUNDARY:
            self.direction *= -1  # Flip direction

        p.resetBasePositionAndOrientation(self.body, new_pos, [0, 0, 0, 1])


class CrowdAvoidanceEnv(gym.Env):
    """Custom Gymnasium Environment for Training a Robot in PyBullet"""
    metadata = {'render.modes': ['human']}

    def __init__(self, use_gui=False):
        super(CrowdAvoidanceEnv, self).__init__()

        # PyBullet Simulation
        self.physics_client = p.connect(p.GUI if use_gui else p.DIRECT) # Use GUI when viewing and don't use when training
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.setTimeStep(1.0 / 240.0)

        # Action and Observation Spaces
        #self.action_space = spaces.Box(low=0.9, high=1.0, shape=(2,), dtype=np.float32)  # Left wheel, Right wheel velocities
        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)  # [v, w]
        num_lidar_readings = 360
        self.observation_space = spaces.Box(
            low=np.array([0, -np.pi, -5, -5, -5, -5, -5]),    # Min: [goal_dist, goal_angle, IMU values]
            high=np.array([5, np.pi, 5, 5, 5, 5, 5]),         # Max: [goal_dist, goal_angle, IMU values]
            dtype=np.float32
        )

        # Load robot URDF
        package_path = ament_index_python.packages.get_package_share_directory('yahboomcar_description')
        urdf_path = os.path.join(package_path, 'urdf', 'MicroROS.urdf')
        self.robot = p.loadURDF(urdf_path, basePosition=[0, 0, 0.05], useFixedBase=False)

        # Goal Position
        self.goal_position = [0, 0, 0.05]

        # Episode constraints
        self.max_steps = EP_LENGTH  # Max steps per episode
        self.current_step = 0  # Track current step count
        

        # Initialize environment
        self.reset()

    def reset(self, seed=None, options=None):
        """Reset the environment and return initial observation."""
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)

        self.previous_linear_speed = 0.0  
        self.previous_angular_speed = 0.0  
        self.previous_goal_distance = None  # Initialize as None, will be set in step()

        # Load ground plane with increased friction
        plane_id = p.loadURDF("plane.urdf")
        #p.changeDynamics(plane_id, -1, lateralFriction=1.0)  # Prevents sliding

        # Reset step count
        self.current_step = 0  

        # Generate random positions for robot and goal
        while True:
            start_x = random.uniform(-ENV_BOUNDARY + 0.5, ENV_BOUNDARY - 0.5)
            start_y = random.uniform(-ENV_BOUNDARY + 0.5, ENV_BOUNDARY - 0.5)

            goal_x = random.uniform(-ENV_BOUNDARY + 0.5, ENV_BOUNDARY - 0.5)
            goal_y = random.uniform(-ENV_BOUNDARY + 0.5, ENV_BOUNDARY - 0.5)

            distance = np.sqrt((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2)
        
            if distance >= MIN_DISTANCE:
                break  # Ensure enough separation

        # Store goal position
        self.goal_position = [goal_x, goal_y, 0.05]
        self.goal_marker = p.loadURDF("sphere_small.urdf", basePosition=self.goal_position, globalScaling=1)
        p.changeVisualShape(self.goal_marker, -1, rgbaColor=[0, 1, 0, 1])

        # Load robot URDF
        package_path = ament_index_python.packages.get_package_share_directory('yahboomcar_description')
        urdf_path = os.path.join(package_path, 'urdf', 'MicroROS.urdf')
        self.robot = p.loadURDF(urdf_path, basePosition=[start_x, start_y, 0.05], useFixedBase=False)

        # Increase friction on all robot parts
        num_joints = p.getNumJoints(self.robot)
        for joint in range(num_joints):
            p.changeDynamics(self.robot, joint, lateralFriction=0.7)  # Prevents wheel slippage'''

        # Reduce drifting by adding velocity damping
        p.changeDynamics(self.robot, -1, lateralFriction=0.7)  

        # Enable better physics parameters for stability
        p.setPhysicsEngineParameter(enableConeFriction=True)  # Better friction handling
        p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)  # Improves contact accuracy

        # Initialize previous goal distance for reward calculation
        self.previous_goal_distance = np.sqrt((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2)

        return self._get_observation(), {}


    def step(self, action):
        """Apply an action and return the next state, reward, done flag."""
        self.current_step += 1  # Increment step count


        # Define robot parameters
        WHEEL_BASE = 0.14  # Distance between left & right wheels, used for left and right control
        MAX_LINEAR_SPEED = 2 # Maximum linear velocity
        MAX_ANGULAR_SPEED = 10 # Maximum angular velocity

        # Scale to max speeds
        linear_speed = action[0] * MAX_LINEAR_SPEED  
        angular_speed = action[1] * MAX_ANGULAR_SPEED  



        '''
        # Compute individual wheel speeds
        #left_wheel_vel = linear_speed - (angular_speed * WHEEL_BASE / 2)
        #right_wheel_vel= linear_speed + (angular_speed * WHEEL_BASE / 2)

        # Move robot using velocity commands
        #p.resetBaseVelocity(self.robot, [linear_speed, 0, 0], [0, 0, angular_speed])
        p.setJointMotorControl2(self.robot, 0, p.VELOCITY_CONTROL, targetVelocity= left_wheel_vel)
        p.setJointMotorControl2(self.robot, 3, p.VELOCITY_CONTROL, targetVelocity=left_wheel_vel)
        p.setJointMotorControl2(self.robot, 1, p.VELOCITY_CONTROL, targetVelocity= -right_wheel_vel)
        p.setJointMotorControl2(self.robot, 2, p.VELOCITY_CONTROL, targetVelocity= -right_wheel_vel)'''

        # Get current robot position and orientation
        TIME_STEP = 1/240.0
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        _, _, yaw = p.getEulerFromQuaternion(orn)  # Extract yaw (Z-axis rotation)

        # Compute new heading based on angular velocity
        new_yaw = yaw + angular_speed * TIME_STEP  # Update yaw with small step
        new_x = pos[0] + linear_speed * np.cos(new_yaw) * TIME_STEP  # Move forward
        new_y = pos[1] + linear_speed * np.sin(new_yaw) * TIME_STEP  # Move forward

        # Apply updated position and orientation
        p.resetBasePositionAndOrientation(self.robot, 
                                        [new_x, new_y, pos[2]], 
                                        p.getQuaternionFromEuler([0, 0, new_yaw]))


        #print(f"Left Wheel Vel: {left_wheel_vel}, Right Wheel Vel: {right_wheel_vel}")
        # Step the simulation
        p.stepSimulation()

        # Get next state
        observation = self._get_observation()

        # Compute reward
        reward, done = self._compute_reward(action)

        # Check if max steps reached
        if self.current_step >= self.max_steps:
            done = True  # End episode if max steps exceeded

        return observation, reward, done, False, {}
    
    def get_goal_angle(self):
        """Return the angle to the goal relative to the robot's heading."""
        robot_pos, robot_ori = p.getBasePositionAndOrientation(self.robot)
        robot_x, robot_y, _ = robot_pos

        goal_dx = self.goal_position[0] - robot_x
        goal_dy = self.goal_position[1] - robot_y
        abs_angle = np.arctan2(goal_dy, goal_dx) # Absolute angle to goal in world frame

        # Relative angle to goal in robot's frame
        _, _, robot_yaw = p.getEulerFromQuaternion(robot_ori)
        relative_angle = abs_angle - robot_yaw

        # Normalize angle to [-pi, pi]
        relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

        return relative_angle

    def _compute_reward(self, action):
        """Calculate the reward function."""

        linear_velocity = action[0]
        angular_velocity = action[1]

        dtg_reward = 0
        htg_reward = 0
        action_reward = 0

        # Get robot position
        robot_pos, _ = p.getBasePositionAndOrientation(self.robot)

        # Distance to goal reward
        goal_dx = self.goal_position[0] - robot_pos[0]
        goal_dy = self.goal_position[1] - robot_pos[1]
        goal_distance = np.sqrt(goal_dx**2 + goal_dy**2)

        previous_distance = getattr(self, "previous_goal_distance", goal_distance)
        distance_improvement = previous_distance - goal_distance
        self.previous_goal_distance = goal_distance  

        if distance_improvement > 0:
            dtg_reward = 1
        if distance_improvement < 0:
            dtg_reward = 0
        
        # Heading to goal reward
        goal_angle = self.get_goal_angle()  # Between -pi and pi
        previous_goal_angle = getattr(self, "previous_goal_angle", goal_angle)

        angle_improvement = (abs(previous_goal_angle) - abs(goal_angle)) * 10.0  # Increase reward for reducing error
        self.previous_goal_angle = goal_angle  

        if angle_improvement > 0:
            htg_reward = 1 
        if angle_improvement < 0:
            htg_reward = 0
        
        # Action reward
        if linear_velocity >= 0 and (((1.0 / 32.0) * -2.0) <= angular_velocity <= (1.0 / 32.0) * 2.0):
            # if linear_speed >= 0 and angular_speed == 0:
            action_reward = 2 #5
        elif linear_velocity >= 0 and angular_velocity > 0:
            action_reward = 1 #1
        elif linear_velocity >= 0 and angular_velocity < 0:
            action_reward = 1 #1


        # ? Collision penalty
        collision_penalty = 0
        contact_points = p.getContactPoints(self.robot)
        if contact_points:
            hit_object_id = contact_points[0][2]
            if hit_object_id != 0 and hit_object_id != getattr(self, "last_collision", None):
                self.last_collision = hit_object_id  
                collision_penalty = -10  
                print(f"Collision detected! Robot hit object {hit_object_id} at {contact_points[0][6:9]}")

        # Goal reached bonus
        goal_bonus = 0
        if goal_distance < 0.2:  
            goal_bonus = 100  
            return goal_bonus, True  

        # Final reward calculation
        reward = (
            dtg_reward + htg_reward + action_reward  
        )

        return reward, False   

    def _get_observation(self):

        """Return LiDAR scan, goal distance, angle, and IMU as observation."""

        #LiDAR scan
        num_readings = 360
        ranges = []

        robot_pos, robot_ori = p.getBasePositionAndOrientation(self.robot)
        robot_x, robot_y, robot_z = robot_pos
        '''
        for angle in np.linspace(-np.pi, np.pi, num_readings):
            direction = [np.cos(angle), np.sin(angle), 0]
            start_pos = [robot_x, robot_y, robot_z + 0.2]  # Raise sensor slightly
            end_pos = [robot_x + direction[0] * 5, robot_y + direction[1] * 5, robot_z + 0.2]
            hit = p.rayTest(start_pos, end_pos)[0]

            distance = hit[2] if hit[0] != -1 else 5.0
            ranges.append(distance)'''

        # Goal distance
        goal_dx = self.goal_position[0] - robot_x
        goal_dy = self.goal_position[1] - robot_y
        goal_distance = np.sqrt(goal_dx**2 + goal_dy**2)
        goal_angle = self.get_goal_angle()
        # IMU
        linear_vel, angular_vel = p.getBaseVelocity(self.robot)
        ax, ay, _ = linear_vel
        wx, wy, wz = angular_vel 
        

        

        return np.array([round(goal_distance, 3), goal_angle, ax, ay, wx, wy, wz], dtype=np.float32)
    



    def render(self, mode="human"):
        """Render the environment."""
        pass  

    def close(self):
        """Shutdown PyBullet simulation."""
        p.disconnect()
