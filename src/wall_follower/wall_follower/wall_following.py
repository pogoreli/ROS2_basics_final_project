import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from custom_interfaces.srv import FindWall
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from rclpy.action import ActionClient
from custom_interfaces.action import OdomRecord

class Wall_following(Node):

    def __init__(self):
        self.publisher_group = ReentrantCallbackGroup()
        self.service_group = ReentrantCallbackGroup()
        self.calibration_group = ReentrantCallbackGroup()
        self.lidar_group = ReentrantCallbackGroup()
        self.action_group = ReentrantCallbackGroup()

        self.wall_found = False
        # Here you have the class constructor
        # call the class constructor
        super().__init__('wall_following')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.publisher_group)
        # create the subscriber object
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.lidar_group)

        # self.subscriber = self.create_subscription(Int32, '/scan_calibration_coefficient', self.laser_callback, QoSProfile(depth=10))
        self.publisher_scan_coeff_ = self.create_publisher(Int32, '/scan_calibration_coefficient', 10, callback_group=self.calibration_group)
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_data = []
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)
        self.action_client = ActionClient(self, OdomRecord, 'record_odom', callback_group=self.action_group)
        self.state = "S0"

        self.linear_velocity_value = 0.19
        self.rotational_velocity_value = 0.3

        self.linear_velocity = 0
        self.rotational_velocity = 0
        self.calibration_coefficient = Int32()

        self.circle_done = False
        self.odom_started = False

        self.find_wall_service = self.create_client(FindWall, 'find_wall', callback_group=self.service_group)
        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.find_wall_service.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        self.req = FindWall.Request()

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_data = msg.ranges

        self.calibration_coefficient.data = 0
        self.publisher_scan_coeff_.publish(self.calibration_coefficient)

    # def find_wall_callback(self, future):
    #     response = future.result()

    #     self.find_wall_service.find_wall_service.destroy()

    #     self.get_logger().info("Wall found")

    def start_action(self):
        self.circle_done = False
        goal_msg = OdomRecord.Goal()
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.action_callback)

    def feedback_callback(self, msg):
        feedback = msg.current_total
        self.get_logger().info("Current distance: " + str(feedback))

    def action_callback(self, future):
        result = future.result().list_of_odoms
        self.get_logger().info("List of odoms: " + str(result))
        self.circle_done = True

        


        
        
    def motion(self):

        while(self.laser_data == []):
            time.sleep(0.5)
            self.get_logger().info("Waiting for laser data")

        if(not self.wall_found):
            self.find_wall()

        # rclpy.spin_until_future_complete(self, self.future, 5.0)

        if(not self.odom_started):
            self.start_action()
            self.odom_started = True

        laser_forward = self.laser_data[359]
        laser_right = self.laser_data[269]

        # print the data
        self.get_logger().info("Forward distance: " + str(laser_forward) + " Right distance: " + str(laser_right))
        
        if(self.circle_done):
            self.state = "S4"
        elif(laser_forward < 0.5):
            self.state = "S3"
        elif(laser_right > 0.3):
            self.state = "S2"
        elif(laser_right < 0.2):
            self.state = "S1"
        else:
            self.state = "S0"

        if self.state == "S0":
            self.state_S0()
        elif self.state == "S1":
            self.state_S1()
        elif self.state == "S2":
            self.state_S2()
        elif self.state == "S3":
            self.state_S3()
        elif self.state == "S4":
            self.state_S4()
        else:
            self.get_logger().info("FSM is broken")

        self.cmd.linear.x = self.linear_velocity
        self.cmd.angular.z = self.rotational_velocity
            
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)

    def find_wall(self):
        self.future = self.find_wall_service.call_async(self.req)
        # self.future.add_done_callback(self.find_wall_callback)
        while rclpy.ok and not self.future.done():
            self.get_logger().info("Searching for a wall")
            time.sleep(0.5)

        self.get_logger().info("Wall found")
        self.wall_found = True

    #S0- go straight
    def state_S0(self):
        self.linear_velocity = self.linear_velocity_value
        self.rotational_velocity = 0.0
        self.get_logger().info("S0")

    #S1- slight left turn
    def state_S1(self):
        self.linear_velocity = self.linear_velocity_value
        self.rotational_velocity = self.rotational_velocity_value
        self.get_logger().info("S1")

    #S2- slight right turn
    def state_S2(self):
        self.linear_velocity = self.linear_velocity_value
        self.rotational_velocity = -self.rotational_velocity_value
        self.get_logger().info("S2")

    #S3- sharp left turn
    def state_S3(self):
        self.linear_velocity = 0.0
        self.rotational_velocity = self.rotational_velocity_value
        self.get_logger().info("S3")

    #S4- stop
    def state_S4(self):
        self.linear_velocity = 0.0
        self.rotational_velocity = 0.0
        self.get_logger().info("S4")


            


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_following = Wall_following()

    executor = MultiThreadedExecutor(num_threads=15)
    executor.add_node(wall_following)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        wall_following.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()