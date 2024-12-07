from enum import Enum
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class FSM_STATES(Enum):
    MANUAL_CONTROL = 'Manual Control'

class FSM(Node):
    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        # Declare and get chair_name parameter
        self.declare_parameter('chair_name', "chair_0")
        chair_name = self.get_parameter('chair_name').get_parameter_value().string_value

        # Publisher for velocity commands
        self._publisher = self.create_publisher(Twist, f"/{chair_name}/cmd_vel", 1)

        # Initial state
        self._cur_state = FSM_STATES.MANUAL_CONTROL

        # Start keyboard control
        self._setup_keyboard_control()

    def _setup_keyboard_control(self):
        """Set up keyboard control using pynput."""
        self.get_logger().info("Keyboard control enabled. Use 'WASD' keys to move, 'SPACE' to stop, and 'Q' to quit.")
        self._listener = keyboard.Listener(on_press=self._on_key_press)
        self._listener.start()

    def _on_key_press(self, key):
        """Callback for keyboard input."""
        try:
            key_char = key.char  # Get the character representation of the key
            twist = Twist()

            if self._cur_state == FSM_STATES.MANUAL_CONTROL:
                if key_char == 'w':  # Forward
                    twist.linear.x = 1.0
                    twist.angular.z = 0.0
                elif key_char == 's':  # Backward
                    twist.linear.x = -1.0
                    twist.angular.z = 0.0
                elif key_char == 'a':  # Turn left
                    twist.linear.x = 0.0
                    twist.angular.z = 1.5
                elif key_char == 'd':  # Turn right
                    twist.linear.x = 0.0
                    twist.angular.z = -1.5
                elif key_char == ' ':  # Stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key_char == 'q':  # Quit
                    self.get_logger().info("Exiting.")
                    rclpy.shutdown()
                    return

                # Publish the Twist message
                self._publisher.publish(twist)

        except AttributeError:
            # Handle special keys if necessary
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

