from isaaclab.app import AppLauncher

# launch omniverse app (has to be running before the other imports)
app_launcher = AppLauncher(headless=True, enable_cameras = True, Livestream = 2) # Livestream 2 opens a gui view 
simulation_app = app_launcher.app


import gymnasium as gym
import torch
import socket
import pickle

import isaaclab_tasks  # noqa: F401 — this registers all envs in the gym registry
from isaaclab_tasks.utils import parse_env_cfg  
import gen3.tasks
import traceback

import json

import isaaclab.sim as sim_utils
from isaaclab.utils.math import quat_apply, quat_mul, quat_conjugate

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
                        
                        output_dict = {"obs": output_obs, "info": output_info}
                        
                    else:
                        buf = recv_exactly(conn, msg_len)
                        octo_action = pickle.loads(buf)
                        

                        octo_action = torch.from_numpy(octo_action)
                        # Isaac sim expects a batch dimension
                        octo_action = octo_action.unsqueeze(0)
                        
                        # Get robot asset
                        robot = env.unwrapped.scene["robot"]
                        body_names = robot.data.body_names
                        ee_body_idx = body_names.index("end_effector_link")

                        # Current orientations
                        root_quat_w = robot.data.root_quat_w.squeeze(0)          # base in world
                        ee_quat_w = robot.data.body_quat_w[:, ee_body_idx].squeeze(0)  # EE in world

                        # EE orientation relative to base: q_b = q_root_w⁻¹ ⊗ q_ee_w
                        ee_quat_b = quat_mul(quat_conjugate(root_quat_w), ee_quat_w)

                        # Octo action: delta in EE local frame
                        delta_pos_ee = torch.tensor(octo_action[:3], device=ee_quat_b.device)
                        # Rotate to base frame
                        delta_pos_base = quat_apply(ee_quat_b, delta_pos_ee)  

                        obs, reward, terminated, truncated, info = env.step(delta_pos_base)

                        # print(obs, reward, terminated, truncated, info)
                        
                        output_obs = replace_tensors(obs["policy"])
                        reward = reward.cpu().numpy()
                        terminated = terminated.cpu().numpy()
                        truncated = truncated.cpu().numpy()
                        info = replace_tensors(info)
                        
                        output_dict = {"obs": output_obs, "reward": reward, "terminated": terminated, "truncated": truncated, "info": info}


                    payload = pickle.dumps(output_dict)
                    header = len(payload).to_bytes(4, 'big')
                    conn.sendall(header+payload)
                    
except Exception as e:
    print(e)
    print(traceback.format_exc())

        
env.close()
simulation_app.close()
