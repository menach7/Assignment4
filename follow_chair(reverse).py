from enum import Enum
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

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
    STARTUP = 'Waiting'
    SLEEPING = 'Sleeping'
    FOLLOWING = 'Following'


class FollowChair(Node):
    def __init__(self):
        super().__init__('FollowChair')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('chair_name', "chair_1")
        self._chair_name = self.get_parameter('chair_name').get_parameter_value().string_value
        self.declare_parameter('target_name', "chair_0")
        self._target_name = self.get_parameter('target_name').get_parameter_value().string_value
        self.get_logger().info(f'Chair {self._chair_name} is following {self._target_name}')

        self.create_subscription(Odometry, f"/{self._chair_name}/odom", self._self_callback, 1)
        self.create_subscription(Odometry, f"/{self._target_name}/odom", self._target_callback, 1)
        self._publisher = self.create_publisher(Twist, f"/{self._chair_name}/cmd_vel", 1)

        self.create_service(SetBool, f"/{self._chair_name}/startup", self._startup_callback)

        self._target_x = None
        self._target_y = None
        self._cur_state = FSM_STATES.STARTUP
        self._run = False

    def _startup_callback(self, request, response):
        self.get_logger().info(f'Got a request {request}')
        if request.data:
            response.success = True
            response.message = "Architecture running"
            self._cur_state = FSM_STATES.FOLLOWING
        else:
            if self._cur_state == FSM_STATES.STARTUP:
                self.get_logger().info('FSM suspended but not yet running.')
                response.success = False
                response.message = "In startup state"
            else:
                self._cur_state = FSM_STATES.SLEEPING
                self._publisher.publish(Twist())
                self.get_logger().info('FSM suspended')
                response.success = True
                response.message = "Architecture suspended"
        return response

    @staticmethod
    def _short_angle(angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        if angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _compute_speed(diff, max_speed, min_speed, gain):
        speed = abs(diff) * gain
        speed = min(max_speed, max(min_speed, speed))
        return math.copysign(speed, diff)

    def _drive_to_target(self, heading0_tol=0.1, range_tol=0.5, safety_distance=1.5):
        twist = Twist()

        x_diff = self._target_x - self._cur_x
        y_diff = self._target_y - self._cur_y
        dist = math.sqrt(x_diff**2 + y_diff**2)

        if dist < safety_distance:
            self.get_logger().info(f'Too close to the target! Reversing.')
            twist.linear.x = -0.5  # Reverse slowly
            twist.angular.z = -1.0
            self._publisher.publish(twist)


        elif dist > range_tol:
            heading = math.atan2(y_diff, x_diff)
            diff = FollowChair._short_angle(heading - self._cur_theta)

            if abs(diff) > heading0_tol:
                twist.angular.z = FollowChair._compute_speed(diff, 5.0, 0.2, 3.0)
            else:
                twist.linear.x = FollowChair._compute_speed(dist, 1.0, 0.1, 0.8)

            self._publisher.publish(twist)
            return False

        self.get_logger().info('At target.')
        self._publisher.publish(twist)
        return True

    def _do_state_at_start(self):
        self.get_logger().info('Waiting in start state.')

    def _do_state_following(self):
        self.get_logger().info('Following the target.')
        if self._target_x is not None:
            self._drive_to_target()
            self._target_x = None
            self._target_y = None

    def _state_machine(self):
        if self._cur_state == FSM_STATES.STARTUP:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.FOLLOWING:
            self._do_state_following()
        elif self._cur_state == FSM_STATES.SLEEPING:
            pass
        else:
            self.get_logger().info(f'Bad state {self._cur_state}')

    def _target_callback(self, msg):
        pose = msg.pose.pose
        self._target_x = pose.position.x
        self._target_y = pose.position.y

    def _self_callback(self, msg):
        pose = msg.pose.pose
        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = FollowChair._short_angle(yaw)
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

