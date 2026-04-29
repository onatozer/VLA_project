"""Launch Isaac Sim Simulator first."""
from isaaclab.app import AppLauncher

# launch omniverse app
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

"""Rest everything follows."""
import gymnasium as gym
import torch
import socket

import isaaclab_tasks  # noqa: F401 — this registers all envs in the gym registry
from isaaclab_tasks.utils import parse_env_cfg  

import json

with open("env_config.json", r) as file:
    configs = json.load(file)

#Task name needs to be passed into botht he parse_env_cfg function and gym.make function
task_name = configs["TASK_NAME"]

env_cfg = parse_env_cfg(**configs)

env = gym.make(task_name, cfg=env_cfg)
obs, info = env.reset()
output_dict = {"obs": obs, "info": info}



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


# TODO: Determine an actual stopping condition (if at all), envs will just run forever if not interrupted
while simulation_app.is_running():
    # Send the environment observation to the octo policy
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()
        with conn:
            payload = json.dumps(output_dict).encode()
            header = len(payload).to_bytes(4, 'big')
            conn.sendall(header+payload)
            
        
    # Read in the action given by Octo
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        header = s.recv_exactly(s,4)
        msg_len = int.from_bytes(header, 'big')
        buf = recv_exactly(s, msg_len)
        
        # output_dict = json.loads(payload)
        
        # Reading in numpy byte buffer into torch directly, should be preffered over json because octo output is always the 7-dim array
        action = torch.frombuffer(buf, dtype=meta['dtype']).reshape(meta['shape'])

    # Execute the action on the environment itself
    obs, reward, terminated, truncated, info = env.step()
    output_dict = {"obs": obs, "reward": reward, "terminated": terminated, "truncated": truncated, "info": info}
    
        
    
env.close()
simulation_app.close()
