# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg
from isaaclab.managers import ObservationTermCfg, SceneEntityCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.sensors import PinholeCameraCfg

##
# Pre-defined configs
##
#TODO: We have 6-dof arm, so this likely has to change
from isaaclab_assets import KINOVA_GEN3_N7_CFG  # isort: skip




##
# Environment configuration
##

#NOTE: Isaac lab also offers 'tiled' camera that's supposedly speeds up image generation, especially if its massively parallel.
# Worth looking at if we start needing speed

@configclass
class BasicCameraParams(CameraCfg):
    spawn = PinholeCameraCfg()
    data_types = ["rgb"]
    height = 256
    width = 256

@configclass
class SceneCameraCfg(BasicCameraParams):
    # Here the camera's placed right at the environment, making it the 'outside' view
    prim_path = "{ENV_REGEX_NS}/Camera"

@configclass
class WristCameraCfg(BasicCameraParams):
    # Here the camera's placed right on the robot's arm, which is the view Octo's most comfortable with. 
    prim_path = "{ENV_REGEX_NS}/Robot/end_effector_link/Camera"



@configclass
class Gen3ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Add camera to the scene that can actually capture visual observations
        self.scene.wrist_camera = WristCameraCfg()

        # Add visual information to the observation
        self.observations.policy.vis_obs_wrist = ObservationTermCfg(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg(name="wrist_camera")}
        ) 
            
        self.observations.policy.concatenate_terms = False

        # switch robot to ur10
        self.scene.robot = KINOVA_GEN3_N7_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)

        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["end_effector_link"]

        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        )

        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "end_effector_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)
