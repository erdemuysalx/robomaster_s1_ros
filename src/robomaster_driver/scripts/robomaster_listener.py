#!/usr/bin/env python

import rospy
from robomaster_msgs.msg import TwistAccel, GimbalAngle
from robomaster import robot, led


class RobomasterListener(robot.Robot):
    def __init__(self, cli=None):
        super().__init__(cli=cli)
        self.initialize(conn_type="sta")
        self.robomaster_ip = rospy.set_param('~robomaster/ip', '192.168.X.X')
        self.robomaster_version = rospy.set_param('~robomaster/version', self.get_version())
        self.robomaster_sn = rospy.set_param('~robomaster/sn', self.get_sn())
        self.chassis_mode = rospy.set_param('~robomaster/chassis_mode', True)
        self.gimbal_mode = rospy.set_param('~robomaster/gimbal_mode', False)
        self.chassis_msg = TwistAccel()
        self.gimbal_msg = GimbalAngle()
        self.sub_chassis = rospy.Subscriber('/robomaster/chassis', TwistAccel, self.cb_chassis)
        self.sub_gimbal = rospy.Subscriber('/robomaster/gimbal', GimbalAngle , self.cb_gimbal)

    def cb_chassis(self, msg):
        rospy.loginfo("[INFO] Subscribed to the chassis")
        if self.chassis_mode == False :
            self.set_robot_mode(mode=robot.CHASSIS_LEAD)
            self.chassis_mode = True
            self.gimbal_mode = False
        self.chassis_msg = msg
        self.chassis.drive_speed(x=self.chassis_msg.twist.linear.x, y=0, z=self.chassis_msg.twist.angular.z, timeout=5)
        self.led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)

    def cb_gimbal(self, msg):
        rospy.loginfo("[INFO] Subscribed to the gimbal")
        if self.gimbal_mode == False:
            self.set_robot_mode(mode=robot.GIMBAL_LEAD)
            self.chassis_mode = False
            self.gimbal_mode = True
        self.gimbal_msg = msg
        self.gimbal.drive_speed(pitch_speed=self.gimbal_msg.pitch_angle, yaw_speed=self.gimbal_msg.yaw_angle)
        self.led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)

if __name__ == '__main__':
    # Initialize ROS node
    print('Initializing robomaster_listener node')
    rospy.init_node('robomaster_listener')
    print('Navigating to robomaster_listener node')
    # Create an instance from listener class
    robomaster_listener = RobomasterListener()
    # Take a loop until an interruption is made
    rospy.spin()
    robomaster_listener.close()