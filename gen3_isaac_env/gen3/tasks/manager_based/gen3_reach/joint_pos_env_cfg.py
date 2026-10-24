# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
import os
import numpy as np

from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.reach.mdp as mdp
from isaaclab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg
from isaaclab.managers import ObservationTermCfg, SceneEntityCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.sensors import PinholeCameraCfg
from isaaclab.assets import RigidObjectCfg, RigidObject
from isaaclab.managers import TerminationTermCfg as DoneTerm
import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg, OperationalSpaceControllerActionCfg, BinaryJointPositionActionCfg
from isaaclab.controllers import DifferentialIKControllerCfg, OperationalSpaceControllerCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Pre-defined configs
##
#TODO: We have 6-dof arm, so this likely has to change
from isaaclab_assets import KINOVA_GEN3_N7_CFG, KINOVA_JACO2_N6S300_CFG  # isort: skip



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
    # Bridge-style third-person view: the camera sits at the far end of the table, elevated, looking back toward the
    # robot base and down at ~40 deg, so the arm reaches "into" the frame from the far side the way it does in Bridge data.
    # (quat = yaw 180 deg about Z, then pitch 40 deg down about Y; "world" convention means the camera looks along its +X)
    # The previous pose, (-0.4, -0.15, 1.2) / (0.906, 0, 0.423, 0), was almost top-down, which Octo never saw in training.
    offset = CameraCfg.OffsetCfg(
        pos=(1.3, 0.0, 0.7),
        rot=(0.0, -0.3420, 0.0, 0.9397),
        convention="world",
    )

@configclass
class WristCameraCfg(BasicCameraParams):
    # Here the camera's placed right on the robot's arm
    prim_path = "{ENV_REGEX_NS}/Robot/end_effector_link/Camera"
    
    # The pre-trained octo policy has an image size for the wrist images hard-coded to 128
    height = 128
    width = 128 
    offset = CameraCfg.OffsetCfg(
        pos=(0.10, 0.0, 0.05),         
        rot = (1.0, 0.0, 0.0, 0.0),
        convention="ros",             
    )

CUBE_SIZE = 0.1

@configclass
class CubeCfg(RigidObjectCfg):
    prim_path="{ENV_REGEX_NS}/Cube"
    init_state=RigidObjectCfg.InitialStateCfg()

    spawn=sim_utils.CuboidCfg(
        size = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE],
        visible = True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(157/255,0.0,1.0), metallic=0.2),
    )


def add_cube_to_scene(cfg: ReachEnvCfg, xy_center, x_range, y_range, cube_cfg: RigidObjectCfg | None = None, size: float = CUBE_SIZE):
    """
    Add a cube to the scene whose position is re-sampled on EVERY reset: `xy_center` (robot base frame, metres) plus a
    uniform offset from `x_range` / `y_range`. The cube rests on the tabletop (z = 0 is the plane the robot base sits
    on, in every Isaac Lab version of the reach env, whereas the table prim's own origin moved between versions).

    Previously the position was sampled with np.random inside __post_init__, i.e. once when the config was built,
    so every episode placed the cube in exactly the same spot.
    """
    cfg.scene.cube = cube_cfg if cube_cfg is not None else CubeCfg()
    cfg.scene.cube.spawn.size = [size, size, size]
    cfg.scene.cube.init_state = RigidObjectCfg.InitialStateCfg(pos=(float(xy_center[0]), float(xy_center[1]), size / 2))

    # reset_root_state_uniform samples an offset from the default (init) root state on each reset
    cfg.events.reset_cube_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": tuple(x_range), "y": tuple(y_range)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("cube"),
        },
    )


# Bridge data was collected at 5 Hz; Octo's actions are per-step deltas at that rate and its observation history
# assumes consecutive frames are 0.2 s apart.
OCTO_CONTROL_DT = 0.2


def configure_for_octo(cfg: ReachEnvCfg):
    """
    Make the reach environment look and behave like the setup Octo was trained on, as far as this scene allows.

    - Turn off the reach task's debug visualization. With it on, the goal-pose and current-pose axis markers (the red /
      green / blue arrows) are rendered INTO the camera images Octo sees.
    - Run control at Bridge's 5 Hz instead of the reach env's 30 Hz. At 30 Hz each 5 Hz delta from Octo was being
      applied six times per intended step, and the IK target moved before the arm could reach it.
    """
    cfg.commands.ee_pose.debug_vis = False

    cfg.decimation = max(1, int(round(OCTO_CONTROL_DT / cfg.sim.dt)))
    cfg.sim.render_interval = cfg.decimation


##
# WidowX 250S: the Bridge dataset robot, set up the way SIMPLER (the Bridge-in-simulation benchmark) sets it up.
# Robot description, joint gains, initial configuration and camera calibration are taken from
# https://github.com/simpler-env/ManiSkill2_real2sim (mani_skill2_real2sim/agents/configs/widowx/defaults.py and
# envs/custom_scenes/base_env.py). The URDF + meshes are vendored under gen3/assets/widowx and converted to USD by
# Isaac Lab on first load (output cached in gen3/assets/widowx/usd, which is git-ignored).
##

WIDOWX_ASSET_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "assets", "widowx"))

WIDOWX_ARM_JOINTS = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
WIDOWX_GRIPPER_JOINTS = ["left_finger", "right_finger"]
WIDOWX_EE_BODY = "ee_gripper_link"   # the Bridge end-effector frame (between the finger tips)
WIDOWX_GRIPPER_OPEN = 0.037          # finger joint limits from the URDF
WIDOWX_GRIPPER_CLOSED = 0.015

WIDOWX250S_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=os.path.join(WIDOWX_ASSET_DIR, "wx250s.urdf"),
        usd_dir=os.path.join(WIDOWX_ASSET_DIR, "usd"),
        fix_base=True,
        # ee_gripper_link hangs off a fixed joint; merging fixed joints would fold it into fingers_link and the IK /
        # reward body lookups would fail.
        merge_fixed_joints=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # SIMPLER's initial configuration for Bridge evaluations: arm up and forward, gripper fully open
        joint_pos={
            "waist":        -0.01840777,
            "shoulder":      0.0398835,
            "elbow":         0.22242722,
            "forearm_roll": -0.00460194,
            "wrist_angle":   1.36524296,
            "wrist_rotate":  0.00153398,
            "left_finger":   WIDOWX_GRIPPER_OPEN,
            "right_finger":  WIDOWX_GRIPPER_OPEN,
        },
    ),
    actuators={
        # Per-joint PD gains and force limits SIMPLER identified to match the real WidowX at 5 Hz
        "arm": ImplicitActuatorCfg(
            joint_names_expr=WIDOWX_ARM_JOINTS,
            effort_limit_sim={"waist": 200.0, "shoulder": 200.0, "elbow": 100.0,
                              "forearm_roll": 100.0, "wrist_angle": 100.0, "wrist_rotate": 100.0},
            velocity_limit_sim=1.5,
            stiffness={"waist": 1169.8, "shoulder": 730.0, "elbow": 808.5,
                       "forearm_roll": 1229.1, "wrist_angle": 1272.3, "wrist_rotate": 1056.3},
            damping={"waist": 330.0, "shoulder": 180.0, "elbow": 152.1,
                     "forearm_roll": 309.6, "wrist_angle": 201.0, "wrist_rotate": 269.5},
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=WIDOWX_GRIPPER_JOINTS,
            effort_limit_sim=60.0,
            velocity_limit_sim=0.12,
            stiffness=1000.0,
            damping=200.0,
        ),
    },
)


@configclass
class BridgeCameraCfg(CameraCfg):
    """
    The Bridge dataset's third-person camera as calibrated by SIMPLER: a Logitech C920 (640x480, fx = fy = 623.6 px,
    i.e. 54.3 deg horizontal FOV) mounted relative to the robot's base_link. Octo was trained on these frames resized
    to 256x256; the client does that resize. "world" convention = camera looks along its +X, same as SAPIEN's camera.
    """
    prim_path = "{ENV_REGEX_NS}/Robot/base_link/Camera"
    data_types = ["rgb"]
    width = 640
    height = 480
    spawn = PinholeCameraCfg(focal_length=20.42, horizontal_aperture=20.955, clipping_range=(0.01, 10.0))
    offset = CameraCfg.OffsetCfg(
        pos=(0.0, -0.16, 0.336),
        rot=(0.909182, -0.0819809, 0.347277, 0.214629),
        convention="world",
    )

@configclass
class Gen3ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Add cameras to the scene that can actually capture visual observations
        self.scene.primary_camera = SceneCameraCfg()
        self.scene.wrist_camera = WristCameraCfg()

        # Add visual information to the observation
        self.observations.policy.vis_obs_wrist = ObservationTermCfg(
            func=mdp.image,
            params={
                "sensor_cfg": SceneEntityCfg(name="wrist_camera"),
                "normalize": False
            }
        ) 
        
        self.observations.policy.vis_obs_primary = ObservationTermCfg(
            func=mdp.image,
            params={
                "sensor_cfg": SceneEntityCfg(name="primary_camera"),
                "normalize": False
            }
        ) 
        

        # Cube somewhere random on the surface of the table, re-sampled every reset (.25 is about as close to the edge as you can get)
        add_cube_to_scene(self, xy_center=self.scene.table.init_state.pos[:2], x_range=(0.0, 0.25), y_range=(0.0, 0.25))


        # Necessary because image obs won't combine properly with the joint position obs
        self.observations.policy.concatenate_terms = False

        # switch robot to ur10
        self.scene.robot = KINOVA_GEN3_N7_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # self.scene.robot = KINOVA_JACO2_N6S300_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)

        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["end_effector_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["end_effector_link"]


        # Just to remove the effect of the pose
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[".*"], 
            body_name="end_effector_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",           
                use_relative_mode=True,       
                ik_method="dls",               # damped least-squares; alternatives: "pinv", "svd", "trans"
                ik_params = {"lambda_val": .2}
            ),
        )
        
        

        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "end_effector_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        configure_for_octo(self)

        # So I don't wait an eternity for debugging
        self.episode_length_s = 12


@configclass
class WidowXBridgeEnvCfg(ReachEnvCfg):
    """
    WidowX 250S on a tabletop with the Bridge camera: the embodiment, viewpoint, control rate and start pose Octo was
    trained on (Bridge V2), following SIMPLER's reproduction of that setup. Registered as "Gen3-Reach-widowx".
    """

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # The Bridge third-person camera, rigidly attached to the robot base like the real one
        self.scene.primary_camera = BridgeCameraCfg()
        # Bridge has no wrist camera; this one is only recorded for debugging and is not given to the policy
        self.scene.wrist_camera = WristCameraCfg(prim_path="{ENV_REGEX_NS}/Robot/" + WIDOWX_EE_BODY + "/Camera")

        self.observations.policy.vis_obs_primary = ObservationTermCfg(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg(name="primary_camera"), "normalize": False},
        )
        self.observations.policy.vis_obs_wrist = ObservationTermCfg(
            func=mdp.image,
            params={"sensor_cfg": SceneEntityCfg(name="wrist_camera"), "normalize": False},
        )
        # Necessary because image obs won't combine properly with the joint position obs
        self.observations.policy.concatenate_terms = False

        # Object workspace as in SIMPLER's Bridge scenes: ~0.31 m in front of the base, +-0.1 m grid.
        # 4 cm cube: fits the gripper (7.4 cm max opening) and matches the size of Bridge objects.
        add_cube_to_scene(self, xy_center=(0.31, 0.0), x_range=(-0.1, 0.1), y_range=(-0.1, 0.1), size=0.04)

        self.scene.robot = WIDOWX250S_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Bridge evaluations start from one fixed arm configuration; no joint randomisation on reset
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)

        # The reach task's own reward / goal machinery is unused by the policy, but it must point at a body that exists
        for term in (self.rewards.end_effector_position_tracking,
                     self.rewards.end_effector_position_tracking_fine_grained,
                     self.rewards.end_effector_orientation_tracking):
            term.params["asset_cfg"].body_names = [WIDOWX_EE_BODY]
        self.commands.ee_pose.body_name = WIDOWX_EE_BODY
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        # Octo's xyz / rpy deltas in the base frame -> differential IK on the 6 arm joints
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=WIDOWX_ARM_JOINTS,
            body_name=WIDOWX_EE_BODY,
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="dls",
            ),
        )

        # Binary gripper action (7th action dim). The host maps Octo's gripper value (1 = open, 0 = closed) onto
        # this term's convention (>= 0 open, < 0 close). Both fingers use positive joint values in this URDF.
        self.actions.gripper_action = BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=WIDOWX_GRIPPER_JOINTS,
            open_command_expr={j: WIDOWX_GRIPPER_OPEN for j in WIDOWX_GRIPPER_JOINTS},
            close_command_expr={j: WIDOWX_GRIPPER_CLOSED for j in WIDOWX_GRIPPER_JOINTS},
        )

        configure_for_octo(self)

        # SIMPLER's Bridge episodes: 60 steps at 5 Hz
        self.episode_length_s = 12


# Backwards-compatible name used by the gym registration
Gen3ReachWidowEnvCfg = WidowXBridgeEnvCfg


@configclass
class GreenCubeCfg(CubeCfg):
    spawn=sim_utils.CuboidCfg(
        size = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE],
        visible = True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0), metallic=0.2),
    )


@configclass
class Gen3FindObjCfg(Gen3ReachEnvCfg):
    """Same as Gen3ReachEnvCfg but with a green cube instead of a purple one."""

    def __post_init__(self):
        super().__post_init__()

        # Replaces the purple cube added by the parent; config holds a cfg, the scene instantiates the RigidObject itself
        add_cube_to_scene(self, xy_center=self.scene.table.init_state.pos[:2], x_range=(0.0, 0.25), y_range=(0.0, 0.25),
                          cube_cfg=GreenCubeCfg())

