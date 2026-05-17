import math
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
import tqdm

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

# from ros2_octo.msg import OctoObservation, OctoAction

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
    rng,
):
    # add batch dim to observations
    observations = jax.tree_map(lambda x: x[None], observations)

    actions = pretrained_model.sample_actions(
        observations,
        tasks,
        rng=rng,
        unnormalization_statistics=pretrained_model.dataset_statistics[
            "bridge_dataset"
        ]["action"],
    )
    # remove batch dim
    return actions[0]


def main(args=None):
    horizon = 2

    # env = KinovaGymEnvironment()
    env = IsaacSimGymEnvironment()
    env = HistoryWrapper(env, horizon)
    env = RHCWrapper(env, 4)
    
    obs, info = env.reset()
    print("-"*10)
    print(obs)
    print(info)
    print("-"*10)
    
    model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
    # print(model.get_pretty_spec())
    # print(model.dataset_statistics)
    print(obs.values())

    # for key,value in obs.items():
    #     print(value.shape)
    #     if key == "timestep_pad_mask":
    #         continue

    #     # Each of the numpy arrays is gonna be organized as (history_dim, batch, im_size, im_size, color), but we need batch dim fist
    #     obs[key] = value.transpose(1,0,2,3,4)
    
    print(obs.keys())
    print(obs["image_wrist"].shape)
    print(f"Num batches {next(iter(obs.values())).shape[0]}")
    
    tasks = model.create_tasks(texts = ["Say hello"])
    
    policy_fn = supply_rng(
        partial(
            sample_actions_isaac,
            model,
        ),
    )

    action = policy_fn(obs, tasks)

    obs, reward, terminated, truncated, info = env.step(action)
    
    action = policy_fn(obs, tasks)
    
    obs, reward, terminated, truncated, info = env.step(action)
    print("-"*20)
    print("File ended")


    


if __name__ == '__main__':
    main()