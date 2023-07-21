# import the SetBool module from std_servs Service interface
from std_srvs.srv import SetBool
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import FindWall

from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Find_wall_service_server(Node):

    def __init__(self):
        # Here you have the class constructor

        self.laser_group = ReentrantCallbackGroup()
        self.service_group = ReentrantCallbackGroup()
        self.publisher_group = ReentrantCallbackGroup()

        self.check_time = 0.1
        self.search_threshold = 2

        # call the class constructor to initialize the node as service_moving
        super().__init__('find_wall_service_server')
        # create the Service Server object
        # defines the type, name, and callback function

        self.laser_data = []

        self.srv = self.create_service(FindWall, 'find_wall', self.FindWall_callback, callback_group=self.service_group)

        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.laser_group)
       
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.publisher_group)
        # create a Twist message
        self.cmd = Twist()    

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_data = msg.ranges
        # self.get_logger().info("Laser forward: " + str(self.laser_data[359]))
        

    def FindWall_callback(self, request, response):
        self.get_logger().info("FindWall_callback")

        smallest_value = self.find_smallest_value_index(self.laser_data)

        while(smallest_value > self.search_threshold and smallest_value < (359 - self.search_threshold)):
            self.get_logger().info("searching the closest wall")
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.linear.x = 0.0
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z = 0.2
            self.publisher_.publish(self.cmd)
            time.sleep(self.check_time)
            smallest_value = self.find_smallest_value_index(self.laser_data)


        while(self.laser_data[359] >= 0.3):
            self.get_logger().info("going toward the wall")
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.linear.x = 0.1
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            time.sleep(self.check_time)

        while(self.laser_data[359] <= 0.25):
            self.get_logger().info("going away from the wall")
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.linear.x = -0.1
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            time.sleep(self.check_time)

        smallest_value = self.find_smallest_value_index(self.laser_data)

        while(smallest_value < (269 - self.search_threshold) or smallest_value > (269 + self.search_threshold)):
            self.get_logger().info("turning 90 degrees")
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.linear.x = 0.0
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            self.cmd.angular.z = 0.2
            self.publisher_.publish(self.cmd)
            time.sleep(self.check_time)
            smallest_value = self.find_smallest_value_index(self.laser_data)

        self.get_logger().info("done")

        # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
        self.cmd.linear.x = 0.0
        # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

        response.wallfound = True
    
        # return the response parameters
        return response

    def find_smallest_value_index(self, list):
        self.get_logger().info("Searching for smallest value")
        smallest = 1000.0
        index = 0

        for i in range(0, 359):
            # self.get_logger().info("Current value is: " + str(list[i]))
            if list[i] < smallest:
                smallest = list[i]
                index = i

        self.get_logger().info("The smallest value is at: " + str(index))

        return index


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    find_wall_service_server = Find_wall_service_server()

    executor = MultiThreadedExecutor()
    executor.add_node(find_wall_service_server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        find_wall_service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()