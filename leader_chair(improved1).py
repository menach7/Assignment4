from enum import Enum
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
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
    AT_START = 'AT STart',
    PERFORMING_TASK = 'Performing Task',
    TASK_DONE = 'Task Done'


class FSM(Node):

    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('chair_name', "chair_0")
        chair_name = self.get_parameter('chair_name').get_parameter_value().string_value

        self.create_subscription(Odometry, f"/{chair_name}/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, f"/{chair_name}/cmd_vel", 1)
        self.create_service(SetBool, f"/{chair_name}/startup", self._startup_callback)
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_id = 0

        # the blackboard
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._points = [[0, 0], [0, 10], [5, 10], [5, 0]]
        self._point = 0
        self._run = False

    def _startup_callback(self, request, resp):
        self.get_logger().info(f'Got a request {request}')
        if request.data:
            self.get_logger().info(f'fsm starting')
            self._run = True
            resp.success = True
            resp.message = "Architecture running"
        else:
            self.get_logger().info(f'fsm suspended')
            self._publisher.publish(Twist())
            self._run = False
            resp.success = True
            resp.message = "Architecture suspended"
        return resp
           

    def _short_angle(angle):
        if angle > math.pi:
            angle = angle - 2 * math.pi
        if angle < -math.pi:
            angle = angle + 2 * math.pi
        assert abs(angle) <= math.pi
        return angle

    def _compute_speed(diff, max_speed, min_speed, gain):
        speed = abs(diff) * gain
        speed = min(max_speed, max(min_speed, speed))
        return math.copysign(speed, diff)
        
    def _drive_to_goal(self, goal_x, goal_y,
                       heading0_tol = 0.10,  # Tighter tolerance for heading
                       range_tol = 0.10):    # Tighter tolerance for distance
        """Return True iff we are at the goal, otherwise drive there"""

        twist = Twist()

        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        if dist > range_tol:
            self.get_logger().info(f'{self.get_name()} driving to goal with goal distance {dist}')
            # turn to the goal
            heading = math.atan2(y_diff, x_diff)
            diff = FSM._short_angle(heading - self._cur_theta)
            if (abs(diff) > heading0_tol):
                twist.angular.z = FSM._compute_speed(diff, 1.0, 0.05, 1.0)  # Increased angular speed for sharper turns
                self.get_logger().info(f'{self.get_name()} turning towards goal heading {heading} current {self._cur_theta} diff {diff} {twist.angular.z}')
                self._publisher.publish(twist)
                self._cur_twist = twist
                return False

            twist.linear.x = FSM._compute_speed(dist, 0.6, 0.2, 0.3)  # Increased speed for straight paths
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} a distance {dist}  from target velocity {twist.linear.x}')
            self._cur_twist = twist
            return False

        self.get_logger().info(f'at goal pose')
        self._publisher.publish(twist)
        return True


    def _do_state_at_start(self):
        self.get_logger().info(f'in start state')
        if self._run:
            self.get_logger().info(f'Starting...')
            self._cur_state = FSM_STATES.PERFORMING_TASK

    def _do_state_performing_task(self):
        if not self._run:
            return
        self.get_logger().info(f'heading to task {self._point}')
        if self._drive_to_goal(self._points[self._point][0], self._points[self._point][1]):
            self._point = self._point + 1
            if self._point >= len(self._points):
                self._point = 0

    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.PERFORMING_TASK:
            self._do_state_performing_task()
        else:
            self.get_logger().info(f'bad state {state_cur_state}')

    def _listener_callback(self, msg):
        pose = msg.pose.pose

        d2 = (pose.position.x - self._last_x) * (pose.position.x - self._last_x) + (pose.position.y - self._last_y) * (pose.position.y - self._last_y)

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = FSM._short_angle(yaw)
        self._state_machine()

        # Log the current position to a text file
        with open('robot_position.txt', 'a') as file:
            file.write(f'{self._cur_x}, {self._cur_y}, {self._cur_theta}\n')
            self.get_logger().info(f'Logged position: {self._cur_x}, {self._cur_y}, {self._cur_theta}')


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

