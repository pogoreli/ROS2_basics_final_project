import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import OdomRecord
from geometry_msgs.msg import Twist
import time
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
import math
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import numpy as np
from geometry_msgs.msg import Point


class Record_odom_action_server(Node):

    def __init__(self):
        super().__init__('record_odom_action_server')
        self.cmd = Twist()
        self.inicial_position = None

        self.group1 = ReentrantCallbackGroup()
        self.group2 = ReentrantCallbackGroup()
        self.group3 = ReentrantCallbackGroup()

        self.total_distance = 0.0

        self._action_server = ActionServer(self, OdomRecord, 'record_odom',self.execute_callback, callback_group=self.group2) 

        self.initialX = 0
        self.initialY = 0
   

        self.position_list = []

        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group1)
    
    
    def odometry_callback(self, msg):
        self.positionX = msg.pose.pose.position.x
        self.positionY = msg.pose.pose.position.y
        
        self.theta = self.euler_from_quaternion(msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.w)


        # self.get_logger().info("X: " + str(self.positionX) + " Y: " + str(self.positionY) + " theta: " + str(self.theta))


    def euler_from_quaternion(self, x, y, z, w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def calculate_distance(self, Xi, Xf, Yi, Yf):
        distanceX = float(Xf - Xi)
        distanceY = float(Yf - Yi)
        distance = math.sqrt((distanceX ** 2) + (distanceY ** 2))
        return distance

        
    
    def execute_callback(self, goal_handle):
        self.initialX = self.positionX
        self.initialY = self.positionY

        previousX = self.positionX
        previousY = self.positionY

        

        distance_from_start = 0.0

        while (distance_from_start <= 0.05):
            time.sleep
            currentX = self.positionX
            currentY = self.positionY
            distance_from_start = self.calculate_distance(self.initialX, currentX, self.initialY, currentY)

        self.get_logger().info("Starting to record the position")

        while (distance_from_start >= 0.05):
            currentX = self.positionX
            currentY = self.positionY
            currentTheta = self.theta

            self.total_distance += self.calculate_distance(previousX, currentX, previousY, currentY)

            feedback = OdomRecord.Feedback()
            feedback.current_total = self.total_distance
            goal_handle.publish_feedback(feedback)

            distance_from_start = self.calculate_distance(self.initialX, currentX, self.initialY, currentY)

            point = Point()
            point.x = currentX
            point.y = currentY
            point.z = currentTheta

            self.position_list.append(point)

            previousX = currentX
            previousY = currentY

            self.get_logger().info("Position recorded. Current distance is: " + str(self.total_distance))

            time.sleep(1)

        self.get_logger().info("Full circle done")

        result = OdomRecord.Result()
        result.list_of_odoms = self.position_list
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    record_odom_action_server = Record_odom_action_server()

    executor = MultiThreadedExecutor()

    executor.add_node(record_odom_action_server)

    executor.spin()

    record_odom_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()