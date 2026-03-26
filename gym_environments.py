import gymnasium as gym
from rclpy import Node
from cv_bridge import CvBridge
import cv2

from trajectory_msgs.msgs import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import Image


class RobotController(Node):
    """ROS2 node that does the task of actually executing the actions given to it by the octo policy"""

    # TODO: Add functionality for the controller to act in the simulation rather than real robot and vice-versa
    def __init__(self, robot_pathname: str = "joint_twist_controller"):
        super.__init__(robot_pathname)

        # Subscribes to the output of the octo policy to be ready to read it's output whenever it's posted.
        # self.octo_subscriber = self.create_subscription(OctoAction, "/octo/action" , self.execute_action_callback, 1)

        # Publisher for updating the actual robot location
        self.location_publisher = self.create_publisher(msg_type = Twist, topic = f"{robot_pathname}/joint_trajectory", qos_profile = 1)
        
        # Subscribers that constantly read the current state and update accordingly
        self.current_pos = {}
        self.create_subscription(JointTrajectoryControllerState, f"{robot_pathname}/controller_state", 
                                                                position_callback, 10)
        
        self.prev_images = []  # Octo takes in last 'x' images, so we'll maintain a list and decide on size later
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.callback, 10)


    def image_callback(self, msg: Image):
        """
           maintain a sliding window of most recently seen images
        """

        # Create delay in how often we're reading in the image
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if dt < 0.1:  # only process at ~10 Hz
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.prev_images.append(cv_image)

        if len(self.prev_images) > 100:
            self.prev_images = self.prev_images[1:]

        self.last_time = now

    def position_callback(self, msg: JointTrajectoryControllerState):
        """
            Update's the node's member variable on the current state of the robot
        """

        actual_pos = {}
        for i, joint_name in enumerate(msg.joint_names):
            joint_pos = msg.actual.positions[i]
            actual_pos[joint_name] = joint_pos

        self.current_pos = actual_pos #NOTE: Thread safe ??

    def execute_step(self, request, response):

        # Convert OctoAction into Twist
        self.logger().info(action)

        # Publish the octo action to the 
        self.location_publisher

        # Update the current state within the gym wrapper

    def execute_reset(self, request, response):


    



class KinovaGymEnvironment(gym.Env):
    """Wrapper that executes the actions on the Kinova arm itself """

    def __init__(self):
        super().__init__()

        # Setup objects needed to control the robot
        self.rclpy = rclpy.init()

        self.node = RobotController()
        
    def _EED_to_joint_position(self, action):
        ...

    def step(self):
        ...

    def reset(self):
        

