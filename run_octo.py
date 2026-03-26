import math

import numpy as np
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

from ros2_octo.msg import OctoObservation, OctoAction

from octo.model.octo_model import OctoModel
from octo.utils.gym_wrappers import HistoryWrapper, TemporalEnsembleWrapper, RHCWrapper
from octo.utils.train_callbacks import supply_rng

from gym_environments import KinovaGymEnvironment

class OctoPolicy(Node):
    """
        ROS2 Node that sends in the given state to the octo policy and sends the action computed by the policy
        Right now this is just purely used for inference
    """

    def __init__(model_path: str = ""):
        super().__init__("octo_policy")

        # If no model path, then just use the pretrained model
        if model_path == "":
            self.policy = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
        
        self.action_publisher = self.create_publisher(OctoAction, "/octo/action", 1)
        
        self.observation_subscriber = self.create_subscription(OctoObservation, "/octo/observation", self.forward_pass_callback, 1)

    def forward_pass(self, msg: OctoObservation):
        task = model.create_task(texts = [msg.task])
        
        # make obs dict from rest of the msg

        response.action = model.sample_actions(
            observation,
            task,
            unnormalization_statistics=model.dataset_statistics["bridge_dataset"]["action"], 
        )

        return response



class ReachPolicy(Node):
    """ROS2 node for controlling a Gen3 robot's reach policy."""
    
    # Define simulation degree-of-freedom angle limits: (Lower limit, Upper limit, Inversed flag)
    SIM_DOF_ANGLE_LIMITS = [
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
        (-360, 360, False),
    ]
    
    # Define servo angle limits (in radians)
    PI = math.pi
    SERVO_ANGLE_LIMITS = [
        (-2 * PI, 2 * PI),
        (-2 * PI, 2 * PI),
        (-2 * PI, 2 * PI),
        (-2 * PI, 2 * PI),
        (-2 * PI, 2 * PI),
        (-2 * PI, 2 * PI),
    ]
    
    # ROS topics and joint names
    STATE_TOPIC = '/joint_twist_controller/controller_state'
    CMD_TOPIC = '/joint_twist_controller/joint_trajectory'

    # ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    #     joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    #     points: [
    #         { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
    #     ]
    # }" -1

    JOINT_NAMES = [
        'joint_1',
        'joint_2',
        'joint_3',
        'joint_4',
        'joint_5',
        'joint_6',
        'joint_7',
    ]
    
    # Mapping from joint name to simulation action index
    JOINT_NAME_TO_IDX = {
        'joint_1': 0,
        'joint_2': 1,
        'joint_3': 2,
        'joint_4': 3,
        'joint_5': 4,
        'joint_6': 5,
        'joint_7': 6
    }

    def __init__(self, fail_quietly: bool = False, verbose: bool = False):
        """Initialize the ReachPolicy node."""
        super().__init__('reach_policy_node')
        
        self.target_command = np.zeros(7)
        self.step_size = 1.0 / 100  # 10 ms period = 100 Hz
        self.timer = self.create_timer(self.step_size, self.step_callback)
        self.i = 0
        self.fail_quietly = fail_quietly
        self.verbose = verbose
        # self.pub_freq = 1.0  # Hz
        self.current_pos = None  # Dictionary of current joint positions
        self.target_pos = None   # List of target joint positions

        # Subscriber for controller state messages
        self.create_subscription(
            JointTrajectoryControllerState,
            self.STATE_TOPIC,
            self.sub_callback,
            10
        )
        
        # Publisher for joint trajectory commands
        self.pub = self.create_publisher(JointTrajectory, self.CMD_TOPIC, 10)
        
        self.get_logger().info("ReachPolicy node initialized.")

    def sub_callback(self, msg: JointTrajectoryControllerState):
        """
        Callback for receiving controller state messages.
        Updates the current joint positions and passes the state to the robot model.
        """
        actual_pos = {}
        for i, joint_name in enumerate(msg.joint_names):
            joint_pos = msg.actual.positions[i]
            actual_pos[joint_name] = joint_pos
        self.current_pos = actual_pos
        
        # Update the robot's state with current joint positions and velocities.
        # TODO:
        # self.get_logger().info(f"Current position of {self.current_pos}")

    #NOTE: consider this function when deploying on real hardware
    def map_joint_angle(self, pos: float, index: int) -> float:
        """
        Map a simulation joint angle (in radians) to the real-world servo angle (in radians).
        
        Args:
            pos: Joint angle from simulation (in radians).
            index: Index of the joint.
        
        Returns:
            Mapped joint angle within the servo limits.
        """
        L, U, inversed = self.SIM_DOF_ANGLE_LIMITS[index]
        A, B = self.SERVO_ANGLE_LIMITS[index]
        angle_deg = np.rad2deg(float(pos))
        # Check if the simulation angle is within limits
        if not L <= angle_deg <= U:
            self.get_logger().warn(
                f"Simulation joint {index} angle ({angle_deg}) out of range [{L}, {U}]. Clipping."
            )
            angle_deg = np.clip(angle_deg, L, U)
        # Map the angle from the simulation range to the servo range
        mapped = (angle_deg - L) * ((B - A) / (U - L)) + A
        if inversed:
            mapped = (B - A) - (mapped - A) + A
        # Verify the mapped angle is within servo limits
        if not A <= mapped <= B:
            raise Exception(
                f"Mapped joint {index} angle ({mapped}) out of servo range [{A}, {B}]."
            )
        return mapped


    def step_callback(self):
        """
        Timer callback to compute and publish the next joint trajectory command.
        """
        if self.current_pos is None:
            self.get_logger().warn("Waiting for robot state...")
            return

        # Set a constant target command for the robot (example values)
        if self.i%3000 < 1000:
            self.target_command = np.array([0.5, 0.0, 0.2, 0.7071, 0.0, 0.7071, 0.0])
        elif self.i%3000 < 2000 and self.i%3000 > 1000:
            self.target_command = np.array([0.4, -0.15, 0.3, 0.7071, 0.0, 0.7071, 0.0])
        else:
            self.target_command = np.array([0.6, 0.1, 0.45, 0.7071, 0.0, 0.7071, 0.0])

    
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_response_callback)
        
        self.i += 1

    def ik_response_callback(self, future):
        """
        Called automatically by the executor when the IK service responds.
        Publishes the resulting joint trajectory.
        """
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            return

        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().warn(f"IK failed with error code: {result.error_code.val}")
            return

        joint_pos = list(result.solution.joint_state.position)

        if len(joint_pos) != 7:
            self.get_logger().error(f"Expected 7 joint positions, got {len(joint_pos)}")
            return

        self.get_logger().info(f"IK output joint positions: {joint_pos}")
        
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = joint_pos
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points.append(point)

        self.pub.publish(traj)
            
        

def main(args=None):
    horizon = 1
    env = KinovaGymEnvironment()

    env = HistoryWrapper(env, horizon)
    env = RHCWrapper(env, 50)
    

    model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
    policy_fn = supply_rng(
        partial(
            model.sample_actions,
            unnormalization_statistics=model.dataset_statistics["action"],
        ),
    )

    


if __name__ == '__main__':
    main()