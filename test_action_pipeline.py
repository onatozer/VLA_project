"""
Open-loop check of the sim action pipeline. No Octo, no JAX: it sends known end-effector deltas to the Isaac host and
reports how far the end effector actually moved, in the robot base frame.

Run it from the Octo container with the Isaac host (env_host.py) already up:

    python3 test_action_pipeline.py

How to read the output, per leg:
    ratio ~ 1.0 and the right axis   -> deltas are honoured; the pipeline is fine and any remaining problem is the policy
    ratio << 1.0                     -> the arm is not reaching the IK target within one control step (actuator gains,
                                        IK damping / lambda_val, or an action scale) and every Octo motion is attenuated
    motion on a different axis       -> frame mismatch between the policy's deltas and the sim's base frame
    no motion at all                 -> the action is not reaching the action manager (dimension mismatch, dropped term)
"""
import numpy as np

from gym_environments import IsaacSimGymEnvironment

STEP = 0.02        # metres per control step: ~2 std of Bridge position deltas, a confident "go this way"
STEPS_PER_LEG = 8  # 8 steps x 2 cm = 16 cm per leg, large enough to be unambiguous, small enough to stay in reach

LEGS = [
    ("+x (forward, away from base)", [STEP, 0.0, 0.0]),
    ("+y (left)",                    [0.0, STEP, 0.0]),
    ("+z (up)",                      [0.0, 0.0, STEP]),
    ("-x (back toward base)",        [-STEP, 0.0, 0.0]),
]


def ee_pos(info):
    return np.asarray(info["ee_pos_b"], dtype=np.float64).reshape(3)


def main():
    env = IsaacSimGymEnvironment(record_obs=False, use_wrist_camera=False)
    obs, info = env.reset()
    start = ee_pos(info)
    print(f"EE start position (base frame): {np.round(start, 3)}")
    print("-" * 72)

    all_ok = True
    for name, delta in LEGS:
        before = ee_pos(info)
        for _ in range(STEPS_PER_LEG):
            # 7-dim like Octo: xyz delta, rpy delta (zero), gripper open (1.0)
            action = np.array(delta + [0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                print("Episode ended mid-test (time limit?) - shorten STEPS_PER_LEG or lengthen episode_length_s")
                return
        after = ee_pos(info)

        commanded = np.array(delta) * STEPS_PER_LEG
        achieved = after - before
        axis = int(np.argmax(np.abs(commanded)))
        ratio = achieved[axis] / commanded[axis]
        off_axis = np.linalg.norm(np.delete(achieved, axis))
        ok = 0.7 < ratio < 1.3 and off_axis < 0.3 * abs(commanded[axis])
        all_ok &= ok

        print(f"{name:32s} commanded {np.round(commanded, 3)}  achieved {np.round(achieved, 3)}  "
              f"on-axis ratio {ratio:5.2f}  off-axis {off_axis:.3f} m  {'OK' if ok else 'PROBLEM'}")

    print("-" * 72)
    print("PIPELINE OK: commanded deltas are reproduced in the sim." if all_ok
          else "PIPELINE PROBLEM: see the legs marked PROBLEM and the key in this file's docstring.")

    env.reset()
    env.close()


if __name__ == "__main__":
    main()
