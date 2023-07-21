import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# sensor_msgs/msg/LaserScan
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import array
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class Laser_calibration(Node):

    def __init__(self):
        self.laser_subscriber_group = ReentrantCallbackGroup()
        self.laser_publisher_group = ReentrantCallbackGroup()
        self.coeff_group = ReentrantCallbackGroup()
        self.wall_found = False
        # Here you have the class constructor
        # call the class constructor
        super().__init__('laser_calibration')
        # create the publisher object
        # self.publisher_ = self.create_publisher(LaserScan, '/scan_calibrated', 10, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.laser_publisher_group)
        self.publisher_ = self.create_publisher(
            LaserScan,
            '/scan_calibrated',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.laser_publisher_group
        )
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.laser_subscriber_group)

        self.subscriber_laser_coeff = self.create_subscription(Int32, '/scan_calibration_coefficient', self.coefficient_callback, QoSProfile(depth=10), callback_group=self.coeff_group)

        # create a LaserScan message
        self.scan_calibrated = LaserScan()

        self.calibration_coefficient = 0
        self.scan_calibration_coefficient = 0

    def coefficient_callback(self, msg):
        self.scan_calibration_coefficient = msg.data


    def laser_callback(self,msg):
        # self.scan_calibrated.ranges = [[None for _ in range(359)]]
        self.calibration_coefficient = self.scan_calibration_coefficient

        self.scan_calibrated.header = msg.header        

        self.scan_calibrated.angle_min = msg.angle_min
        self.scan_calibrated.angle_max = msg.angle_max
        self.scan_calibrated.angle_increment = msg.angle_increment

        self.scan_calibrated.time_increment = msg.time_increment

        self.scan_calibrated.scan_time = msg.scan_time

        self.scan_calibrated.range_min = msg.range_min
        self.scan_calibrated.range_max = msg.range_max

        ranges = array.array('f', [0.0] * 360)
        intensities = array.array('f', [0.0] * 360)

        for i in range(0, 360):
            new_angle = self.find_new_angle(i, self.calibration_coefficient)
            self.get_logger().info('New angle: ' + str(new_angle) + " Angle: " + str(i) + " Calibration coefficient: " + str(self.calibration_coefficient))

            # self.get_logger().info('New angle: ' + str(new_angle) + " Angle: " + str(i) + " Length of previous array: " + str(len(msg.ranges)) + " Length of new array: " + str(len(ranges)))
            ranges[new_angle] = msg.ranges[i]
            intensities[new_angle] = msg.intensities[i]

        self.get_logger().info(str(msg.ranges))
        self.get_logger().info(str(ranges))
        
        self.scan_calibrated.ranges = ranges
        self.scan_calibrated.intensities = intensities

        self.publisher_.publish(self.scan_calibrated)




    def find_new_angle(self, current_angle, calibration_coefficient):
        min_angle = 0
        max_angle = 359
        new_angle = current_angle + calibration_coefficient

        if(new_angle > max_angle):
            while (new_angle > max_angle):
                new_angle = new_angle - max_angle
        elif(new_angle < min_angle):
            new_angle = math.abs(new_angle)

            if new_angle > max_angle:
                new_angle = new_angle % max_angle

            new_angle = max_angle - new_angle
            
        return new_angle



# def main(args=None):
#     rclpy.init(args=args)
#     laser_calibration = Laser_calibration()
#     rclpy.spin(laser_calibration)
#     rclpy.shutdown()


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    laser_calibration = Laser_calibration()

    executor = MultiThreadedExecutor()
    executor.add_node(laser_calibration)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        laser_calibration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()