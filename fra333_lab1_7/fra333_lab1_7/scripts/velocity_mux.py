#!/usr/bin/python3

# import all other neccesary libraries
from std_msgs.msg import Float64
import sys
#add more libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityMux(Node):
    def __init__(self):

        #use 'super()' method to make it possible to access the members of a parent class
        super().__init__('velocity_mux')

        # get the rate from argument or default
        if len(sys.argv)>2: 
            self.rate = float(sys.argv[1])
        else:
            self.rate = 5.0

        #create subscriber
        self.__subscriptions1 = self.create_subscription(Float64, '/linear/noise', self.linear_vel_sub_callback, 10)
        self.__subscriptions2 = self.create_subscription(Float64, '/angular/noise', self.angular_vel_sub_callback, 10)
        # create publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)

        # create timer
        self.timer = self.create_timer(1/self.rate, self.timer_callback)

        # additional attributes
        self.linearvel = 0.0
        self.angularvel = 0.0
        self.get_logger().info(f'Starting {self.get_name()}')

    def linear_vel_sub_callback(self,msg:Float64):
        self.linearvel = msg.data
    
    def angular_vel_sub_callback(self,msg:Float64):
        self.angularvel = msg.data
    
    def timer_callback(self):
        #the message type is geometry_msgs/Twist
        msg = Twist()
        #message data: linear
        msg.linear.x = self.linearvel
        #message data: angular
        msg.angular.z = self.angularvel
        #let the publisher publish the message
        self.publisher_.publish(msg)

def main(args=None):
    #run the class through object
    rclpy.init(args=args)
    velocity_muxed = VelocityMux()
    rclpy.spin(velocity_muxed)
    velocity_muxed.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
