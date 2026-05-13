from isaaclab.app import AppLauncher

# launch omniverse app (has to be running before the other imports)
app_launcher = AppLauncher(headless=True, enable_cameras = True)
simulation_app = app_launcher.app


import gymnasium as gym
import torch
import socket

import isaaclab_tasks  # noqa: F401 — this registers all envs in the gym registry
from isaaclab_tasks.utils import parse_env_cfg  
import gen3.tasks


import json

# Probably overkill to be doing this, but leaving it for now
with open("env_config.json", "r") as file:
    configs = json.load(file)

#Task name needs to be passed into botht he parse_env_cfg function and gym.make function
task_name = "Gen3-Reach-v0"

env_cfg = parse_env_cfg(**configs)

env = gym.make(task_name, cfg=env_cfg)
# print(gym.observation_space)
obs, info = env.reset()
# print(output_dict)


# Port value for isaac sim container is hard-coded to 6,000 and octo container 5,000. There's probably a better practice, but for this case its fine
ISAACSIM_PORT = 6_000
OCTO_PORT = 5_000

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

# TODO: Determine an actual stopping condition (if at all), envs will just run forever if not interrupted, maybe that's also fine
while simulation_app.is_running():
    # Send the environment observation to the octo policy
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, ISAACSIM_PORT))
        print("binded") 
        s.listen(1)
        print("listened") 

        # TODO: Think of a stopping condition (if at all) for this
        while(True):
            conn, addr = s.accept()
            print("Accepted connection")

            with conn:
                # The pattern for interacting with the Octo policy is going to be just repeatingly recieving the data it sends,
                # and sending back the environment observation to it
                
                header = recv_exactly(conn,4)
                msg_len = int.from_bytes(header, 'big')

                # We're gonna interperent msg_len of 0 as a reset call, because if no action is set, all that's to be done is to reset the environment
                if msg_len == 0:
                    obs,info = env.reset()
                    output_dict = {"obs": obs, "info": info}
                    
                else:
                    buf = recv_exactly(conn, msg_len)
                # action = torch.frombuffer(buf, dtype=meta['dtype']).reshape(meta['shape'])


                print(f"Sending output dict of {output_dict}")
                payload = json.dumps(output_dict).encode()
                header = len(payload).to_bytes(4, 'big')
                conn.sendall(header+payload)

        
env.close()
simulation_app.close()
