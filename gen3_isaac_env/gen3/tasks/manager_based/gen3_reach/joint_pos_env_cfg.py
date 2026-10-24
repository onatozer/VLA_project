# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math
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


def add_cube_to_scene(cfg: ReachEnvCfg, spread: float, cube_cfg: RigidObjectCfg | None = None, size: float = CUBE_SIZE):
    """
    Add a cube to the scene whose position is re-sampled on EVERY reset, uniformly within `spread` meters of the
    table origin in x and y. The cube rests on the table surface (center is half a cube-height above the table).

    Previously the position was sampled with np.random inside __post_init__, i.e. once when the config was built,
    so every episode placed the cube in exactly the same spot.
    """
    cfg.scene.cube = cube_cfg if cube_cfg is not None else CubeCfg()
    cfg.scene.cube.spawn.size = [size, size, size]

    # x/y from the table prim; z from the robot mounting plane (z = 0 is the tabletop the robot base sits on, in every
    # Isaac Lab version of the reach env, whereas the table prim's own origin moved between versions).
    table_x, table_y, _ = cfg.scene.table.init_state.pos
    cfg.scene.cube.init_state = RigidObjectCfg.InitialStateCfg(pos=(float(table_x), float(table_y), size / 2))

    # reset_root_state_uniform samples an offset from the default (init) root state on each reset
    cfg.events.reset_cube_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, spread), "y": (0.0, spread)},
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


WIDOWX250_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        # /workspace/isaaclab/source/isaaclab_assets/data/Robots/Interbotix/widowx250.usd
        usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Robots/Interbotix/widowx250.usd",  # local or Nucleus path
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            fix_root_link=True,  # <-- add this
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "waist":        0.0,
            "shoulder":     0.0,
            "elbow":        0.0,
            "forearm_roll": 0.0,
            "wrist_angle":  0.0,
            "wrist_rotate": 0.0,  
            "left_finger":  0.015,
            "right_finger": -0.037,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["waist", "shoulder", "elbow",
                              "forearm_roll", "wrist_angle", "wrist_rotate"],
            effort_limit_sim=10.0,
            velocity_limit_sim=3.14,
            stiffness=800.0,
            damping=40.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*finger.*"],
            effort_limit_sim=5.0,
            velocity_limit_sim=1.0,
            stiffness=200.0,
            damping=10.0,
        ),
    },
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
        add_cube_to_scene(self, spread=0.25)


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
class Gen3ReachWidowEnvCfg(ReachEnvCfg):

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Add cameras to the scene that can actually capture visual observations
        self.scene.primary_camera = SceneCameraCfg()
        self.scene.wrist_camera = WristCameraCfg(prim_path = "{ENV_REGEX_NS}/Robot/wx250s_ee_gripper_link/Camera")

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
        

        # Cube somewhere random on the surface of the table, re-sampled every reset (.4 is about as close to the edge as you can get).
        # 4 cm so it fits in the WidowX gripper (~7 cm max opening) and is the size of the objects in Bridge data.
        add_cube_to_scene(self, spread=0.4, size=0.04)


        # Necessary because image obs won't combine properly with the joint position obs
        self.observations.policy.concatenate_terms = False

        # Use the Widowx robot from the octo training samples 
        self.scene.robot = WIDOWX250_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)

        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["wx250s_ee_gripper_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["wx250s_ee_gripper_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["wx250s_ee_gripper_link"]

        # Differential IK action for the WidowX 250S
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"],
            body_name="wx250s_ee_gripper_link",
            controller=DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=True,
                ik_method="dls",
            ),
        )

        # Binary gripper action (7th action dim). The host maps Octo's gripper value (1 = open, 0 = closed) onto
        # this term's convention (>= 0 open, < 0 close). Finger targets match the WidowX 250S joint range.
        self.actions.gripper_action = BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["left_finger", "right_finger"],
            open_command_expr={"left_finger": 0.037, "right_finger": -0.037},
            close_command_expr={"left_finger": 0.015, "right_finger": -0.015},
        )

        # override command generator body
        self.commands.ee_pose.body_name = "wx250s_ee_gripper_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)

        configure_for_octo(self)

        self.episode_length_s = 5


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
        add_cube_to_scene(self, spread=0.25, cube_cfg=GreenCubeCfg())

