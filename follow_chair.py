from enum import Enum
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw.
    """
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class FSM_STATES(Enum):
    STARTUP = "Waiting"
    SLEEPING = "Sleeping"
    FOLLOWING = "Following"

class FollowChair(Node):
    def __init__(self):
        super().__init__('FollowChair')
        self.get_logger().info(f'{self.get_name()} created')

        # Parameters
        self.declare_parameter('chair_name', "chair_1")
        self._chair_name = self.get_parameter('chair_name').get_parameter_value().string_value
        self.declare_parameter('target_name', "chair_0")
        self._target_name = self.get_parameter('target_name').get_parameter_value().string_value

        self.get_logger().info(f'Chair {self._chair_name} is following {self._target_name}')

        # Subscriptions and Publishers
        self.create_subscription(Odometry, f"/{self._chair_name}/odom", self._self_callback, 1)
        self.create_subscription(Odometry, f"/{self._target_name}/odom", self._target_callback, 1)
        self._publisher = self.create_publisher(Twist, f"/{self._chair_name}/cmd_vel", 1)

        # Service
        self.create_service(SetBool, f"/{self._chair_name}/startup", self._startup_callback)

        # State variables
        self._target_x, self._target_y = None, None
        self._cur_x, self._cur_y, self._cur_theta = None, None, None
        self._cur_state = FSM_STATES.STARTUP

    def _startup_callback(self, request, resp):
        if request.data:
            resp.success = True
            resp.message = "Architecture running"
            self._cur_state = FSM_STATES.FOLLOWING
        else:
            self._cur_state = FSM_STATES.SLEEPING
            self._publisher.publish(Twist())
            resp.success = True
            resp.message = "Architecture suspended"
        return resp

    @staticmethod
    def _short_angle(angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _distance_based_speed(self, distance, outer_threshold, inner_threshold, max_speed, min_speed):
        if distance > outer_threshold:
            return max_speed
        elif distance > inner_threshold:
            return min_speed + (max_speed - min_speed) * ((distance - inner_threshold) / (outer_threshold - inner_threshold))
        else:
            return min_speed

    def _drive_to_target(self, heading0_tol=0.15, range_tol=1.0, safety_distance=0.5):
        """
        Drives to the target while avoiding collisions by adjusting speed based on proximity.
        """
        if self._target_x is None or self._cur_x is None:
            return

        twist = Twist()
        x_diff = self._target_x - self._cur_x
        y_diff = self._target_y - self._cur_y
        dist = math.sqrt(x_diff**2 + y_diff**2)

        # Avoid getting too close
        if dist < safety_distance:
            self.get_logger().info(f'Too close to the target! Stopping.')
            self._publisher.publish(Twist())  # Stop the robot
            return True

        if dist > range_tol:
            heading = math.atan2(y_diff, x_diff)
            diff = self._short_angle(heading - self._cur_theta)

            if abs(diff) > heading0_tol:
                twist.angular.z = self._distance_based_speed(abs(diff), math.pi, 0.1, 0.5, 0.1)
            else:
                twist.linear.x = self._distance_based_speed(dist, 5.0, 2.0, 0.5, 0.1)

            self._publisher.publish(twist)
            self.get_logger().info(f'Driving to target: Distance={dist}, Linear={twist.linear.x}, Angular={twist.angular.z}')
            return False

        self.get_logger().info(f'At target.')
        self._publisher.publish(Twist())
        return True

    def _do_state_following(self):
        if self._target_x is not None:
            self._drive_to_target()

    def _state_machine(self):
        if self._cur_state == FSM_STATES.FOLLOWING:
            self._do_state_following()

    def _target_callback(self, msg):
        self._target_x = msg.pose.pose.position.x
        self._target_y = msg.pose.pose.position.y

    def _self_callback(self, msg):
        pose = msg.pose.pose
        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = self._short_angle(yaw)
        self._state_machine()

def main(args=None):
    rclpy.init(args=args)
    node = FollowChair()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
