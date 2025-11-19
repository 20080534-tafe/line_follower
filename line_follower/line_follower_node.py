import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
import math

class AvoidWallNode(Node):
    def __init__(self):
        super().__init__('avoid_wall')

        # Declare parameters for the node that can be changed in params yaml
        self.declare_parameter('forward_speed', 3.0)
        self.declare_parameter('turn_speed', 2.0)
        self.declare_parameter('wall_threshold', 0.05)

        # Get the parameters
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.wall_threshold = self.get_parameter('wall_threshold').value

        # Subscribe to the /ps7 to receive data
        self.sensor_sub = self.create_subscription(
            Range,
            '/ps7',
            self.sensor_callback,
            10
        )

        # Publisher to send velocity commands to the robot
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Create some variables for the robot
        self.obstacle_detected = False
        self.last_sensor_value = 0.0
        self.turning = False
        self.turn_start_time = None
        self.turn_duration = None

        # Create a timer that calls the control_loop method every 0.1 second
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Avoid wall node started.")  # Indicate that the node started

    def sensor_callback(self, msg):
        """Callback function to process the sensor data.""" # Added a docstring
        self.last_sensor_value = msg.range  # Store the sensor value
        self.get_logger().info(f"Sensor value: {self.last_sensor_value} m")  # Log the sensor value

        # If the sensor detects an obstacle within the distance, set the flag to True
        if self.last_sensor_value < self.wall_threshold:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False  # No obstacle detected

