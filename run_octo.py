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
from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper, RHCWrapper
from octo.utils.train_callbacks import supply_rng

from gym_environments import KinovaGymEnvironment
from functools import partial
import jax
import jax.numpy as jnp
import tensorflow_datasets as tfds


def main(args=None):
    horizon = 1

    env = KinovaGymEnvironment()
    env = HistoryWrapper(env, horizon)
    env = RHCWrapper(env, 1)
    
    obs, info = env.reset()
    print("-"*10)
    print(obs)
    print("-"*10)
    
    model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
    print(model.get_pretty_spec())
    # print(model.dataset_statistics)
    

    # Dummy obs for right now, lifted straight from octo git
    builder = tfds.builder_from_directory(builder_dir='gs://gresearch/robotics/bridge/0.1.0/')
    ds = builder.as_dataset(split='train[:1]')

    # sample episode + resize to 256x256 (default third-person cam resolution)
    episode = next(iter(ds))
    steps = list(episode['steps'])
    images = [cv2.resize(np.array(step['observation']['image']), (256, 256)) for step in steps]

    # extract goal image & language instruction
    goal_image = images[-1]
    language_instruction = steps[0]['observation']['natural_language_instruction'].numpy().decode()
    task = model.create_tasks(texts=[language_instruction])                  # for language conditioned
    

    pred_actions, true_actions = [], []
    WINDOW_SIZE = 2
    for step in tqdm.trange(len(images) - (WINDOW_SIZE - 1)):
        input_images = np.stack(images[step:step+WINDOW_SIZE])[None]
        observation = {
            'image_primary': input_images,
            'timestep_pad_mask': np.full((1, input_images.shape[1]), True, dtype=bool)
        }
        
        # this returns *normalized* actions --> we need to unnormalize using the dataset statistics
        actions = model.sample_actions(
            observation, 
            task, 
            unnormalization_statistics=model.dataset_statistics["bridge_dataset"]["action"], 
            rng=jax.random.PRNGKey(0)
        )
        actions = actions[0] # remove batch dim

    pred_actions.append(actions)
    final_window_step = step + WINDOW_SIZE - 1
    true_actions.append(np.concatenate(
        (
            steps[final_window_step]['action']['world_vector'], 
            steps[final_window_step]['action']['rotation_delta'], 
            np.array(steps[final_window_step]['action']['open_gripper']).astype(np.float32)[None]
        ), axis=-1
    ))

    action = pred_actions[0]
    print(action.shape)

    env.step(action[0])
    
    # policy_fn = supply_rng(
    #     partial(
    #         model.sample_actions, 
    #         unnormalization_statistics = model.dataset_statistics[ "bridge_dataset" ]["action"])
    #     )

    # # Dummy obs/task just to make sure that the thing actually works
    # goal_image = jnp.zeros((256, 256, 3), dtype=np.uint8)
    # goal_instruction = "Move the block"

    # task = model.create_tasks(texts=[goal_instruction])
    
    # # action = np.array(policy_fn(observation= goal_image, tasks= task, rng = jax.random.PRNGKey(0),), dtype=np.float64)
    # actions = policy_fn(jax.tree_map(lambda x: x[None], obs), task)
    # actions = actions[0]
    # print(action, action.shape)


    


if __name__ == '__main__':
    main()