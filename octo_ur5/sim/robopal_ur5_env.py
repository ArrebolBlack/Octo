import numpy as np
import logging
from robopal.envs import RobotEnv
from robopal.wrappers import GymWrapper
import gym
from robopal.robots import UR5e

logging.basicConfig(level=logging.INFO)

# Initialize the Robopal environment
env = RobotEnv(
    robot="UR5e",
    render_mode='human',
    control_freq=200,
    is_interpolate=False,
    controller="CARTIK",
    is_show_camera_in_cv=True,
    is_render_camera_offscreen=True,
    # camera_in_render='cam'  #ValueError: The camera "cam" does not exist.
)

#TODO:
     ################################################################################
        # Define observation and action spaces
env.observation_space = gym.spaces.Box(
    low=-np.inf, high=np.inf, shape=(env.calculate_obs_dim(),), dtype=np.float64
)
# UR5e.pos_max_bound = np.array([0.6, 0.2, 0.37])
# UR5e.pos_min_bound = np.array([0.3, -0.2, 0.12])
UR5e.action_low_bound = np.array([-1, -1, -1, -1, -1, -1, -1])
UR5e.action_high_bound = np.array([1, 1, 1, 1, 1, 1, 1])
env.action_space = gym.spaces.Box(
   # low=env.robot.action_space_low, high=env.robot.action_space_high, shape=(env.calculate_action_dim(),), dtype=np.float64
   # low=env.min_action, high=env.max_action, shape=(env.calculate_action_dim(),), dtype="float64"
    low=UR5e.action_low_bound, high=UR5e.action_high_bound, shape=(env.calculate_action_dim(),), dtype="float64"

)

def calculate_obs_dim(self):
    # Define the dimension of the observation space based on your environment
    return 14  # Example value; modify according to your environment's observation

def calculate_action_dim(self):
    # Define the dimension of the action space based on your environment
    return 7  # Example value; modify according to your environment's action
##########################################################################################
env.min_action = UR5e.action_low_bound
env.max_action = UR5e.action_high_bound
env.max_episode_steps = 200
        
env = GymWrapper(env)

# Check observation and action spaces
print("Observation Space:", env.observation_space)
print("Action Space:", env.action_space)

# Check configuration
configs = env.get_configs()
print("Environment Configurations:", configs)

# Run a test episode
env.reset()

print(hasattr(env, 'step'))


for _ in range(10):  # Short episode for testing
    action = env.action_space.sample()
    print("Action:", action)
    obs, reward, done, truncated, info = env.step(action)
    print("Observation:", obs)
    print("Reward:", reward)
    print("Done:", done)
    print("Truncated:", truncated)
    if done or truncated:
        break
env.close()
