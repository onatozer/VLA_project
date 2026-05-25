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
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.sensors import PinholeCameraCfg
from isaaclab.assets import RigidObjectCfg, RigidObject
from isaaclab.managers import TerminationTermCfg as DoneTerm
import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg, OperationalSpaceControllerActionCfg
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
    offset = CameraCfg.OffsetCfg(
        pos=(-0.4, -0.15, 1.2),      # Above the table/robot
        # rot=(0.707, 0.0, 0.707, 0.0),  # +90° around Y: +X → −Z (down)
        rot = (0.906, 0.000, 0.423, 0.000),
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

@configclass 
class CubeCfg(RigidObjectCfg):
    prim_path="{ENV_REGEX_NS}/Cube"
    init_state=RigidObjectCfg.InitialStateCfg()
    
    spawn=sim_utils.CuboidCfg(
        size = [.1,.1,.1],
        visible = True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(157/255,0.0,1.0), metallic=0.2),
    )


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
        

        self.scene.cube = CubeCfg()

        # Set the starting position of the cube to be somewhere random on the surface of the table
        table_surface = np.array(self.scene.table.init_state.pos)
        starting_pos = np.random.rand(3) * .25 # .2 is about as close to the edge of the table as you can get
        starting_pos[2] = 0 

        starting_pos = tuple(table_surface + starting_pos)
        # print(f"CUBE STARTING POS: {starting_pos}")

        self.scene.cube.init_state = RigidObjectCfg.InitialStateCfg(pos = starting_pos)

        
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
        

        self.scene.cube = CubeCfg()

        # Set the starting position of the cube to be somewhere random on the surface of the table
        table_surface = np.array(self.scene.table.init_state.pos)
        starting_pos = np.random.rand(3) * .4 # .4 is about as close to the edge of the table as you can get
        starting_pos[2] = 0 

        starting_pos = tuple(table_surface + starting_pos)

        self.scene.cube.init_state = RigidObjectCfg.InitialStateCfg(pos = starting_pos)

        
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

        # override command generator body
        self.commands.ee_pose.body_name = "wx250s_ee_gripper_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)
        
        self.episode_length_s = 5


@configclass
class Gen3FindObjCfg(Gen3ReachEnvCfg):




    def __post_init__(self):
        super().__post_init__()

        # cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/FrankaEmika/panda_instanceable.usd")
        # prim_path = "{ENV_REGEX_NS}/World/Table/Asset"

        # # use the `func` reference in the config class
        # cfg.func(prim_path, cfg)
        
        cube_cfg = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/",
            spawn=sim_utils.CuboidCfg(
                        size = [.1,.1,.1],
                        visible = True,
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                        mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                        collision_props=sim_utils.CollisionPropertiesCfg(),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0), metallic=0.2
                    ),
                    init_state=RigidObjectCfg.InitialStateCfg(),
                    ),
            )

        self.scene.cube = RigidObject(cfg=cone_cfg)

