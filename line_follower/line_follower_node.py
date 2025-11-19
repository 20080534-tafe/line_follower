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

    def control_loop(self):
        """This method runs in a loop every 0.1 seconds to control the robot's movement."""
        twist_stamped = TwistStamped()  # Create a new TwistStamped message to send velocity commands
        twist_stamped.header.stamp = self.get_clock().now().to_msg()  # Add timestamp to the message
                                                                      # Getting the time was a pain in the ass
        twist_stamped.header.frame_id = "base_link"  # Set the frame to "base_link"
                                                     # I just had to google this because it wouldnt work without a header for me

        if self.obstacle_detected and not self.turning:
            # If an obstacle is detected and the robot is not currently turning, start turning
            self.turning = True
            self.turn_start_time = self.get_clock().now()  # Record the start time of the turn
            self.turn_duration = (math.pi / 2) / self.turn_speed  # Calculate how to turn 90 degrees
                                                                  # This was also a pain in the ass, for some reason math.pi worked really well
            self.get_logger().info(f"Obstacle detected, starting 90-degree turn.")

        if self.turning:
            # If the robot is turning, calculate the time elapsed since it started turning
            elapsed_time = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9  # This was the only way i could find to convert into seconds

            if elapsed_time >= self.turn_duration:
                # If the turn duration has passed, stop turning and move forward again
                self.turning = False
                twist_stamped.twist.linear.x = self.forward_speed  # Set forward speed
                twist_stamped.twist.angular.z = 0.0  # Stop turning
                self.get_logger().info(f"Turn completed, moving forward.") # Logging more shit
            else:
                # If the robot is still turning, keep turning
                twist_stamped.twist.linear.x = 0.0  # No forward movement while turning
                twist_stamped.twist.angular.z = self.turn_speed  # Set turn speed
                self.get_logger().info(f"Turning... {elapsed_time:.2f} sec / {self.turn_duration:.2f} sec.") # Logging more shit
        else:
            # If there's no obstacle and the robot is not turning, move forward
            twist_stamped.twist.linear.x = self.forward_speed  # Set forward speed
            twist_stamped.twist.angular.z = 0.0  # No turning
            self.get_logger().info(f"Path clear, moving forward.") # Idk if we even need this much more output

        # Publish the velocity command to the robot
        self.cmd_pub.publish(twist_stamped)
