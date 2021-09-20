#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from robomaster import robot, chassis, gimbal, led


class RobomasterListener:
    def __init__(self, gimbalMode=True, chassisMode=False ):
        self.robot = robot.Robot()  # Robomaster robot instance
        self.robot.initialize(conn_type="sta")
        self.chassis = self.robot.chassis
        self.gimbal = self.robot.gimbal
        self.led = self.robot.led
        self.chassis_data = None
        self.gimbal_data = None
        self.chassisMode = chassisMode
        self.gimbalMode = gimbalMode
        self.velocity_sub = rospy.Subscriber('/robomaster/chassis', Twist, self.chassis_callback)
        self.gimbal_sub = rospy.Subscriber('/robomaster/gimbal', Twist , self.gimbal_callback)

    def chassis_callback(self, msg):
        self.chassis_data = msg

        if self.chassisMode == False :
            self.robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
            self.chassisMode = True
            self.gimbalMode = False

        self.chassis.drive_speed(x=self.chassis_data.linear.x, y=0, z=self.chassis_data.angular.x, timeout=5)
        self.led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)

    def gimbal_callback(self, msg):
        self.gimbal_data = msg

        if self.gimbalMode == False:
            self.robot.set_robot_mode(mode=robot.GIMBAL_LEAD)
            self.chassisMode = False
            self.gimbalMode = True
        
        self.gimbal.drive_speed(pitch_speed=self.gimbal.linear.x, yaw_speed=self.gimbal.angular.x)
        self.led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('robomaster_listener')
    # Create an instance from listener class
    robomaster_listener = RobomasterListener()
    # Take a loop until an interruption is made
    rospy.spin()