import numpy as np

from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, RHCWrapper
from octo.utils.train_callbacks import supply_rng

from gym_environments import KinovaGymEnvironment, IsaacSimGymEnvironment
from functools import partial
import jax

# The pretrained Octo checkpoints were trained on this dataset, and the policy's raw output is normalized with its statistics.
# We un-normalize with the same statistics so the actions are real end-effector deltas (meters / radians) and a gripper value.
ACTION_STATS_DATASET = "bridge_dataset"


# create policy functions
def sample_actions_isaac(
    pretrained_model: OctoModel,
    observations,
    tasks,
    rng):
    # add batch dim to observations
    observations = jax.tree_map(lambda x: x[None], observations)

    # `unnormalization_statistics` applies mean/std only to the dimensions flagged in the statistics' mask, so the gripper
    # dimension (which is not normalized during training) is left untouched. Doing this by hand with mean/std over all
    # 7 dims would corrupt the gripper value.
    actions = pretrained_model.sample_actions(
        observations,
        tasks,
        unnormalization_statistics=pretrained_model.dataset_statistics[ACTION_STATS_DATASET]["action"],
        rng=rng,
    )
    # remove batch dim -> (pred_horizon, action_dim)
    return np.asarray(actions[0])


def main(args=None):
    horizon = 2       # number of past observations fed to the policy
    exec_horizon = 4  # number of actions from each predicted chunk that get executed before re-planning

    # env = KinovaGymEnvironment()
    # Bridge has no wrist camera, so Octo gets the third-person image only (the wrist video is still recorded)
    env = IsaacSimGymEnvironment(record_obs = True, use_wrist_camera = False)
    env = HistoryWrapper(env, horizon)
    env = RHCWrapper(env, exec_horizon)

    obs, info = env.reset()
    print("-"*10)
    print({k: getattr(v, "shape", v) for k, v in obs.items()})
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

    terminated = False
    truncated = False
    step = 0
    while not (terminated or truncated):
        # Every action chunk goes through the same (un-normalized) path, and is computed from the latest observation
        action = policy_fn(obs, tasks)
        obs, reward, terminated, truncated, info = env.step(action)
        step += 1
        print(f"step {step:4d} | action[0] = {np.round(action[0], 4)} | reward = {reward}")

    if truncated:
        print("Episode truncated (time limit reached)")
    else:
        print("Episode terminated")

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
