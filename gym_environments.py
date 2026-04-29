import gymnasium as gym
import numpy as np
from cv_bridge import CvBridge
from queue import Queue
import rclpy
from rclpy.node import Node
from rclpy.task import Future
import threading

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image


class SnapshotQueue(Queue):
    """
    Helper class that allows us to view the current queue in a thread-safe manner, as suggeted by 
    (https://stackoverflow.com/questions/16686292/how-to-get-the-items-in-queue-without-removing-the-items)
    """
    
    def __init__(self, maxsize: int = 10):
        super().__init__(maxsize)

    def snapshot(self):
        with self.mutex:
            return list(self.queue)


class RobotController(Node):
    """ROS2 node that does the task of actually executing the actions given to it by the octo policy"""

    # TODO: Add functionality for the controller to act in the simulation rather than real robot and vice-versa
    def __init__(self, robot_pathname: str = "kinova_arm"):
        
        super().__init__(robot_pathname)
        
        self.get_logger().info("Testing logger")

        # Subscribes to the output of the octo policy to be ready to read it's output whenever it's posted.
        # self.octo_subscriber = self.create_subscription(OctoAction, "/octo/action" , self.execute_action_callback, 1)

        # Publisher for updating the actual robot location
        self.location_publisher = self.create_publisher(msg_type = TwistStamped, topic = f"twist_controller/commands", qos_profile = 10)
        
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
        
        self.prev_images = SnapshotQueue()
        
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.get_logger().debug("Finished initializing RobotController node")


    def image_callback(self, msg: Image):
        """
           maintain a sliding window of most recently seen images
        """
        self.get_logger().debug(f"Running image callback on images buffer of size {self.prev_images.qsize()}")

        # Create delay in how often we're reading in the image
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if dt < 0.1:  # only process at ~10 Hz
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.prev_images.put(cv_image)

        self.last_time = now

    def execute_step(self, action):
        self.action = action

        self.get_logger().debug(f"Recieved action of {action} from octo policy")

        # Start the arm-mover clock, and freeze the node until it reaches the destination
        # self.future = Future()
        # self.arm_move_timer = self.create_timer(0.05, self._move_arm) # 20 Hz
        # self.get_logger().debug(f"{self.future.done()}")
        # rclpy.spin_until_future_complete(self, self.future)

        self._move_done_event = threading.Event()

        self.arm_move_timer = self.create_timer(0.05, self._move_arm)
        
        # Block main thread until _move_arm signals completion
        self._move_done_event.wait() #NOTE: Can add a timeout so that thread won't freeze if we don't reach the destination
       
        # Output the images of the current state
        return self.prev_images.snapshot()

    def _move_arm(self):
        t = self.tf_buffer.lookup_transform('base_link', 'tool_frame', rclpy.time.Time())
        
        current = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ])

        self.get_logger().debug(f"Arm currently at position {t}")

        # TODO: Also decide how we're gonna handle angular rotation

        # indicies from 0-2 of the action deal with the EED position 
        error = self.action[0:3] - current
        distance = np.linalg.norm(error)
        # self.get_logger().debug(f"err_vec {type(error)} Distance {type(distance)}")
        # Error <class 'jaxlib.xla_extension.ArrayImpl'> Distance <class 'numpy.float32'>

        msg = TwistStamped()

        if distance < self.tolerance:
            # Arm has reached the destination
            self.location_publisher.publish(msg)
            self.arm_move_timer.destroy()
            self._move_done_event.set() # Let the thread running this know we're done and it can stop blocking

        else:
            #NOTE: We are not handling the orientation of the gripper itself rn
            velocity =  error / distance  
            velocity = np.clip(velocity, -0.1, 0.1)  # cap at 10 cm/s 
            self.get_logger().debug(f"Velocity {velocity}")
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(velocity[0])
            msg.twist.linear.y = float(velocity[1])
            msg.twist.linear.z = float(velocity[2])
            self.get_logger().debug(f"Publishing twist message {msg}")
            self.location_publisher.publish(msg)

    def execute_reset(self):
        #TODO: Probably want to decide on some type of 'starting position' for the arm and reset it back to that positon on reset
        self.get_logger().debug(f"Resetting Kinova arm. Image queue has size {self.prev_images.qsize()}")

        
        return self.prev_images.snapshot()


class KinovaGymEnvironment(gym.Env):
    """Wrapper that executes the actions on the Kinova arm itself """

    def __init__(self):
        super().__init__()

        # Setup objects needed to control the robot
        
        # Only init if not already initialized (safety for multiple envs)
        if not rclpy.ok():
            rclpy.init()
            
        rclpy.logging.set_logger_level('kinova_arm', rclpy.logging.LoggingSeverity.DEBUG)
        self.node = RobotController()

        self.observation_space = gym.spaces.Dict({
            "image_primary": gym.spaces.Box(
                low=0,
                high=255,
                shape=(256, 256, 3),
                dtype=np.uint8,
            ),
        })
        
        # Keep the robot controller node constantly spinning in seperate thread so that latest images are properly updated
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon = True)
        self._spin_thread.start()

        print("Finished initializing the threads")

        self._is_closed = False
                

    def step(self, action):
        obs = {}

        images = self.node.execute_step(action)
        obs["image_primary"] = images
        
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        return obs, reward, terminated, truncated, info

    def reset(self, seed, options): # arguments carry no meaning here, but still including so program doesn't error
        obs = {}

        images = self.node.execute_reset()
        obs["image_primary"] = images
        
        info = {}
        return obs, info

    def close(self):
        """Gym API method - called automatically when env is deleted or manually"""
        if self._is_closed:
            return
            
        self._is_closed = True
        
        # 1. Destroy the node (stops accepting callbacks)
        if self.node:
            self.node.destroy_node()
            
        # 2. Shutdown ROS (this makes rclpy.spin() return in the thread)
        if rclpy.ok():
            rclpy.shutdown()
            
        # 3. Wait for thread to die (with timeout so we don't hang)
        if hasattr(self, '_spin_thread') and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
            
        super().close()
        print("Close finished")

    def __del__(self):
        """Destructor - safety net in case close() wasn't called"""
        print("Delete is being called")
        self.close()
        

class IsaacSimEnvironment(gym.Env):
    """
        Class that basically acts as a wrapper over the socket commands that send the Octo actions to the actual IsaacSim environment
        setup in another docker container
    """

    def __init__(self):
        super().__init__()

    def _recv_exactly(self, sock, n):
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


    def step(self, action):
        # Jax has no native way to convert to bytes, so just convert to numpy first
        buffer = np.array(action).tobytes()

        # Send action to environment
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen(1)
            conn, addr = s.accept()

            with conn:
                buffer = json.dumps(output_dict).encode()
                header = len(buffer).to_bytes(4, 'big')
                conn.sendall(header+buffer)
        

        # Recieve response from the environment
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen(1)
            conn, addr = s.accept()

            header = self._recv_exactly(s, 4)
            msg_len = int.from_bytes(header, "big")
            payload = self._recv_exactly(s, msg_len)

            gym_output = json.loads(payload)

        obs = gym_output["obs"]
        reward = gym_output["reward"]
        terminated = gym_output["terminated"]
        truncated = gym_output["truncated"]
        info = gym_output["info"]

        return gym_output.values()
            



    # Does this even make sense ??
    def reset(self):
        ...


