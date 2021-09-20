#!/usr/bin/env python

import rospy
from robomaster_msgs import TwistAccel, GimbalAngle
from robomaster import robot, chassis, gimbal, led


class RobomasterListener:
    def __init__(self, gimbalMode=True, chassisMode=False ):
        self.robot = robot.Robot()  # Robomaster robot instance
        self.robot.initialize(conn_type="sta")
        self.chassis = self.robot.chassis
        self.gimbal = self.robot.gimbal
        self.led = self.robot.led
        self.twist_accel = None
        self.gimbal_angle = None
        self.chassisMode = chassisMode
        self.gimbalMode = gimbalMode
        self.velocity_sub = rospy.Subscriber('/robomaster/chassis', TwistAccel, self.chassis_callback)
        self.gimbal_sub = rospy.Subscriber('/robomaster/gimbal', GimbalAngle , self.gimbal_callback)

    def chassis_callback(self, msg):
        self.twist_accel = msg

        if self.chassisMode == False :
            self.robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
            self.chassisMode = True
            self.gimbalMode = False

        self.chassis.drive_speed(x=self.twist_accel.twist.linear.x, y=0, z=self.twist_accel.twist.angular.z, timeout=5)
        self.led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)

    def gimbal_callback(self, msg):
        self.gimbal_angle = msg

        if self.gimbalMode == False:
            self.robot.set_robot_mode(mode=robot.GIMBAL_LEAD)
            self.chassisMode = False
            self.gimbalMode = True
        
        self.gimbal.drive_speed(pitch_speed=self.gimbal_angle.pitch, yaw_speed=self.gimbal_angle.yaw)
        self.led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('robomaster_listener')
    # Create an instance from listener class
    robomaster_listener = RobomasterListener()
    # Take a loop until an interruption is made
    rospy.spin()