import math
import numpy as np
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

# from ros2_octo.msg import OctoObservation, OctoAction

from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper, RHCWrapper
from octo.utils.train_callbacks import supply_rng

from gym_environments import KinovaGymEnvironment
from functools import partial
import jax
import jax.numpy as jnp


def main(args=None):
    horizon = 1

    env = KinovaGymEnvironment()
    env = HistoryWrapper(env, horizon)
    env = RHCWrapper(env, 50)
    
    obs, info = env.reset()
    print("-"*10)
    print(obs)
    print("-"*10)
    

    model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
    print(model.get_pretty_spec())
    # print(model.dataset_statistics)
    
    policy_fn = supply_rng(
        partial(
            model.sample_actions, 
            unnormalization_statistics = model.dataset_statistics[ "bridge_dataset" ]["action"])
        )

    # Dummy obs/task just to make sure that the thing actually works
    goal_image = jnp.zeros((256, 256, 3), dtype=np.uint8)
    goal_instruction = "Move the block"

    task = model.create_tasks(texts=[goal_instruction])
    
    # action = np.array(policy_fn(observation= goal_image, tasks= task, rng = jax.random.PRNGKey(0),), dtype=np.float64)
    actions = policy_fn(jax.tree_map(lambda x: x[None], obs), task)
    actions = actions[0]
    print(action, action.shape)


    


if __name__ == '__main__':
    main()