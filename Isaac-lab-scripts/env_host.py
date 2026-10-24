from isaaclab.app import AppLauncher

# launch omniverse app (has to be running before the other imports)
app_launcher = AppLauncher(headless=True, enable_cameras = True, Livestream = 2) # Livestream 2 opens a gui view 
simulation_app = app_launcher.app


import gymnasium as gym
import numpy as np
import torch
import socket
import pickle

import isaaclab_tasks  # noqa: F401 — this registers all envs in the gym registry
from isaaclab_tasks.utils import parse_env_cfg  
import gen3.tasks
import traceback

import json

import isaaclab.sim as sim_utils
from isaaclab.utils.math import quat_from_euler_xyz, axis_angle_from_quat, subtract_frame_transforms
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(filename='isaac_sim.log', encoding='utf-8', level=logging.INFO)
logger.setLevel("INFO") # Set to lowest level so all levels are allowed


#Task name needs to be passed into both the parse_env_cfg function and gym.make function
task_name = "Gen3-Reach-v0"

env_cfg = parse_env_cfg(task_name = task_name, num_envs = 1)

# env_cfg.viewer.resolution = (640, 480)
# env_cfg.viewer.eye = (2.0, 2.0, 2.0)      # Camera position
# env_cfg.viewer.lookat = (0.0, 0.0, 0.5)   # Look-at point


env = gym.make(task_name, cfg=env_cfg, render_mode = "rgb_array")

print(sim_utils.queries.get_all_matching_child_prims("/"))


video_kwargs = {
    "video_folder": "./video_recordings",
    "episode_trigger": lambda ep: ep % 1 == 0,
    "video_length": 10_000,
    "name_prefix": task_name,
    # "disable_logger": True,
}

env = gym.wrappers.RecordVideo(env, **video_kwargs)

obs, info = env.reset()
# print(output_dict)


# Port value for isaac sim container is hard-coded to 6,000 and octo container 5,000. There's probably a better practice, but for this case its fine
ISAACSIM_PORT = 6_000

HOST = "0.0.0.0"



# Helper function for the socket stuff
def recv_exactly(sock, n):
    """Receive exactly n bytes from socket, or raise."""
    chunks = []
    received = 0
    while received < n:
        chunk = sock.recv(min(65536, n - received))
        if not chunk:
            raise ConnectionError(f"Expected {n} bytes, got {received}")
        chunks.append(chunk)
        received += len(chunk)
    return b''.join(chunks)


# Total action dimension the Isaac environment expects: 6 for the differential-IK arm term alone,
# 7 if a binary gripper action term is also configured.
ENV_ACTION_DIM = env.unwrapped.action_manager.total_action_dim
_warned_about_gripper = False


def octo_to_isaac_action(octo_action: torch.Tensor) -> torch.Tensor:
    """
    Convert a single un-normalized Octo / Bridge action into the command the Isaac env's action manager expects.

    Octo (trained on Bridge) outputs: [dx, dy, dz, droll, dpitch, dyaw, gripper]
      - position / rotation deltas are end-effector deltas expressed in the ROBOT BASE frame
      - gripper is in [0, 1], with 1 = open and 0 = closed

    Isaac Lab's DifferentialInverseKinematicsAction in relative "pose" mode expects:
      - [dx, dy, dz] in the robot base frame (it works with the EE pose relative to the root), so no frame rotation needed
      - [rx, ry, rz] as an axis-angle rotation vector, not euler angles
    and BinaryJointPositionAction expects a scalar where < 0 => close, >= 0 => open.
    """
    global _warned_about_gripper

    delta_pos = octo_action[0:3]

    # Euler (roll, pitch, yaw) delta -> quaternion -> axis-angle rotation vector
    rpy = octo_action[3:6]
    delta_quat = quat_from_euler_xyz(rpy[0:1], rpy[1:2], rpy[2:3])   # shape (1, 4)
    delta_rotvec = axis_angle_from_quat(delta_quat).squeeze(0)       # shape (3,)

    arm_cmd = torch.cat([delta_pos, delta_rotvec])

    if ENV_ACTION_DIM >= 7:
        gripper_open = octo_action[6] > 0.5
        gripper_cmd = torch.where(gripper_open, torch.tensor(1.0, device=octo_action.device), torch.tensor(-1.0, device=octo_action.device))
        cmd = torch.cat([arm_cmd, gripper_cmd.reshape(1)])
    else:
        if not _warned_about_gripper:
            logger.warning(f"Environment action dim is {ENV_ACTION_DIM}; no gripper action term configured, gripper command dropped")
            _warned_about_gripper = True
        cmd = arm_cmd

    return cmd


_EE_BODY_NAME = env.unwrapped.cfg.actions.arm_action.body_name


def ee_pose_in_base():
    """
    End-effector position (m) and quaternion (w, x, y, z) expressed in the robot base frame, the same frame the
    policy's deltas are interpreted in. Shipped to the client in `info` so motion can be checked against commands.
    """
    robot = env.unwrapped.scene["robot"]
    ee_idx = robot.find_bodies(_EE_BODY_NAME)[0][0]
    ee_pos_w = robot.data.body_pos_w[:, ee_idx]
    ee_quat_w = robot.data.body_quat_w[:, ee_idx]
    pos_b, quat_b = subtract_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, ee_pos_w, ee_quat_w)
    return pos_b[0].cpu().numpy(), quat_b[0].cpu().numpy()


# Helper function that replaces all the tensors in a dictionary with numpy arrays
def replace_tensors(input_dict: dict):
    output_dict = {}
    for key, val in input_dict.items():
        if isinstance(val, dict):
            output_dict[key] = replace_tensors(val)
        
        elif torch.is_tensor(val):
            val = val.cpu().numpy()
            output_dict[key] = val
        else:
            output_dict[key] = val
        
    return output_dict


# TODO: Determine an actual stopping condition (if at all), envs will just run forever if not interrupted, maybe that's also fine
try:
    while simulation_app.is_running():
        # Send the environment observation to the octo policy
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, ISAACSIM_PORT))
            print("binded") 
            s.listen(10)
            print("listened") 

            # TODO: Think of a stopping condition (if at all) for this
            while(True):
                conn, addr = s.accept()
                print("Accepted connection")
                octo_action = -1

                with conn:
                    # The pattern for interacting with the Octo policy is going to be just repeatingly recieving the data it sends,
                    # and sending back the environment observation to it
                    
                    header = recv_exactly(conn,4)
                    msg_len = int.from_bytes(header, byteorder = 'big', signed = True)

                    if msg_len == -1:
                        raise Exception("Manual break called")
                        break

                    # We're gonna interperent msg_len of 0 as a reset call, because if no action is set, all that's to be done is to reset the environment
                    if msg_len == 0:
                        obs, info = env.reset()
                        
                        # Convert all the torch tensors into numpy arrays to allow the pickle library to steralize the object
                        output_obs = replace_tensors(obs["policy"])
                        output_info = replace_tensors(info)
                        output_info["ee_pos_b"], output_info["ee_quat_b"] = ee_pose_in_base()

                        output_dict = {"obs": output_obs, "info": output_info}
                        
                    else:
                        buf = recv_exactly(conn, msg_len)
                        octo_action = pickle.loads(buf)
                        
                        octo_action = torch.from_numpy(np.asarray(octo_action)).float().to(env.unwrapped.device).reshape(-1)

                        isaac_action = octo_to_isaac_action(octo_action)

                        logger.info(f"octo action:  {octo_action.cpu().numpy()}")
                        logger.info(f"isaac action: {isaac_action.cpu().numpy()}")

                        # Isaac sim expects a batch dimension
                        isaac_action = isaac_action.unsqueeze(0)
                        obs, reward, terminated, truncated, info = env.step(isaac_action)

                        # print(obs, reward, terminated, truncated, info)
                        
                        output_obs = replace_tensors(obs["policy"])
                        reward = reward.cpu().numpy()
                        terminated = terminated.cpu().numpy()
                        truncated = truncated.cpu().numpy()
                        info = replace_tensors(info)
                        info["ee_pos_b"], info["ee_quat_b"] = ee_pose_in_base()
                        logger.info(f"ee pos (base): {info['ee_pos_b']}")

                        output_dict = {"obs": output_obs, "reward": reward, "terminated": terminated, "truncated": truncated, "info": info}


                    payload = pickle.dumps(output_dict)
                    header = len(payload).to_bytes(4, 'big')
                    conn.sendall(header+payload)
                    
except Exception as e:
    print(e)
    print(traceback.format_exc())

        
env.close()
simulation_app.close()
