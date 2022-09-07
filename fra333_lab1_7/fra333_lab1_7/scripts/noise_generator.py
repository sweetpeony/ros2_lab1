#!/usr/bin/python3

# import all other neccesary libraries here
import sys
#add more libraries
from std_msgs.msg import Float64
import numpy as np
import rclpy
from rclpy.node import Node
#import service
from lab1_interfaces.srv import SetNoise

class NoiseGenerator(Node):

    def __init__(self):

        #use 'super()' method to make it possible to access the members of a parent class
        super().__init__('noise_generator')

        # get the rate from argument or default
        if len(sys.argv)>2: 
            self.rate = float(sys.argv[1])
        else:
            self.rate = 5.0

        # declare parameter
        self.declare_parameters(namespace = '', parameters = [('no_param',None)])

        #create publisher
        self.publisher_ = self.create_publisher(Float64, f'{self.get_namespace()}/noise', 10)

        #create timer
        self.timer = self.create_timer(1/self.rate, self.timer_callback)

        #create service
        self.set_noise_service = self.create_service(SetNoise,f'{self.get_namespace()}/set_noise',self.set_noise_callback)

        self.mean = 0.0
        self.variance = 1.0

        # additional attributes
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()} with the default parameter. mean: {self.mean}, variance: {self.variance}')
    
    def set_noise_callback(self,request:SetNoise.Request,response:SetNoise.Response):
        #request mean and variance data
        self.mean = request.mean.data
        self.variance = request.variance.data
        return response
    
    def timer_callback(self):
        #the message type is std_msgs/Float64
        msg = Float64()
        #message data is random noise from normal distribution
        msg.data = np.random.normal(self.mean, np.sqrt(self.variance))
        #let the publisher publish the message
        self.publisher_.publish(msg)

def main(args=None):
    #run the class through object
    rclpy.init(args=args)
    noise = NoiseGenerator()
    rclpy.spin(noise)
    noise.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
