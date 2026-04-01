import gymnasium as gym
from cv_bridge import CvBridge
# import cv2
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class RobotController(Node):
    """ROS2 node that does the task of actually executing the actions given to it by the octo policy"""

    # TODO: Add functionality for the controller to act in the simulation rather than real robot and vice-versa
    def __init__(self, robot_pathname: str = "kinova_arm"):
        rclpy.init()
        
        super().__init__(robot_pathname)

        # Subscribes to the output of the octo policy to be ready to read it's output whenever it's posted.
        # self.octo_subscriber = self.create_subscription(OctoAction, "/octo/action" , self.execute_action_callback, 1)

        # Publisher for updating the actual robot location
        self.location_publisher = self.create_publisher(msg_type = Twist, topic = f"{robot_pathname}/cmd_vel", qos_profile = 10)
        
        # Subscribers that constantly read the current state and update accordingly
        # self.current_pos = {}
        # self.create_subscription(JointTrajectoryControllerState, f"{robot_pathname}/controller_state", 
                                                                # position_callback, 10)
                                                                
        # TF listener to get current end-effector pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #TODO: Determine actual reasonable values for these 
        self.tolerance = 0.005   # 5mm stopping threshold
        # self.arm_move_timer = self.create_timer(0.05, self._move_arm, auto_start = False)  # 20 Hz
        self.future = Future()
        
        self.prev_images = []  # Octo takes in last 'x' images, so we'll maintain a list and decide on size later
        
        # Is this really needed ??
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)


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

    # def position_callback(self, msg: JointTrajectoryControllerState):
    #     """
    #         Update's the node's member variable on the current state of the robot
    #     """

    #     actual_pos = {}
    #     for i, joint_name in enumerate(msg.joint_names):
    #         joint_pos = msg.actual.positions[i]
    #         actual_pos[joint_name] = joint_pos

    #     self.current_pos = actual_pos #NOTE: Thread safe ??

    def execute_step(self, request, response):
        self.action = request.action

        self.logger().debug(f"Recieved action of {action} from octo policy")

        # Start the arm-mover clock, and freeze the node until it reaches the destination
        self.arm_move_timer = self.create_timer(0.05, self._move_arm)  # 20 Hz
        self.spin_until_future_complete(self, self.future)
       
        # Output the images of the current state
        response.images = self.prev_images

    def _move_arm(self):
        t = self.tf_buffer.lookup_transform('base_link', 'tool_frame', rclpy.time.Time())
        
        current = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ])

        self.get_logger.debug(f"Arm currently at position {t}")

        error = self.action - current
        distance = np.linalg.norm(error)

        msg = Twist()

        if distance < self.tolerance:
            # Arm has reached the destination
            self.location_publisher.publish(msg)
            self.arm_move_timer.destory()
            self.future.set_result(True)

        else:
            #NOTE: We are not handling the orientation of the gripper itself rn
            velocity =  error / dist  
            velocity = np.clip(velocity, -0.1, 0.1)  # cap at 10 cm/s 
            msg.linear.x = velocity[0]
            msg.linear.y = velocity[1]
            msg.linear.z = velocity[2]
            self.location_publisher.publish(msg)

    def execute_reset(self, request, response):
        ...



class KinovaGymEnvironment(gym.Env):
    """Wrapper that executes the actions on the Kinova arm itself """

    def __init__(self):
        super().__init__()

        # Setup objects needed to control the robot
        self.node = RobotController()

        self.observation_space = gym.spaces.Dict({
            "image_primary": gym.spaces.Box(
                low=0,
                high=255,
                shape=(256, 256, 3),
                dtype=np.uint8,
            ),
        })
                

    def step(self, action):
        obs = {}

        images = self.node.execute_step(action)
        obs["image_primary"] = images
        ...

    def reset(self):
        ...
        

