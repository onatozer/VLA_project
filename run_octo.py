import math
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import tqdm
import socket
import random

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist


from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper, RHCWrapper, ResizeImageWrapper
from octo.utils.train_callbacks import supply_rng

from gym_environments import KinovaGymEnvironment, IsaacSimGymEnvironment
from functools import partial
import jax
import jax.numpy as jnp
import tensorflow_datasets as tfds

# create policy functions
def sample_actions_isaac(
    pretrained_model: OctoModel,
    observations,
    tasks,
    rng):
    # add batch dim to observations
    observations = jax.tree_map(lambda x: x[None], observations)

    actions = pretrained_model.sample_actions(
        observations,
        tasks,
        rng=rng,
    )
    # remove batch dim
    return actions[0]


def main(args=None):
    horizon = 2

    # env = KinovaGymEnvironment()
    env = IsaacSimGymEnvironment(record_obs = True)
    env = HistoryWrapper(env, horizon)
    env = RHCWrapper(env, 4)
    
    obs, info = env.reset()
    print("-"*10)
    print(obs)
    print(info)
    print("-"*10)
    
    model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
    tasks = model.create_tasks(texts = ["Pick up the purple cube"])
    
    policy_fn = supply_rng(
        partial(
            sample_actions_isaac,
            model,
        ),
    )

    action = policy_fn(obs, tasks)
    action = action * model.dataset_statistics["bridge_dataset"]["action"]["std"][None] + model.dataset_statistics["bridge_dataset"]["action"]["mean"][None]
    obs, reward, terminated, truncated, info = env.step(action)
    
    while(not terminated and not truncated):
        if truncated:
            print("To long :-( ")
        obs, reward, terminated, truncated, info = env.step(action)
        action = policy_fn(obs, tasks)
        # action = [random.random() for i in range(7)]

    env.reset()
        
    print("-"*20)
    print("File ended")
    
    # Send signal to server for it to peacefully stop
    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

    #     s.connect(("172.18.0.1", 6_000))
    #     break_msg = (-1).to_bytes(4, byteorder = "big", signed = True)
    #     s.sendall(break_msg)


    


if __name__ == '__main__':
    main()