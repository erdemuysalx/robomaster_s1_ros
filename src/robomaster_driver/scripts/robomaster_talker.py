#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
from robomaster import robot


class RobomasterTalker(robot.Robot):
    def __init__(self, cli=None):
        super().__init__(cli=cli)
        self.initialize(conn_type="sta")
        self.image_msg = Image()
        self.imu_msg = Imu()
        self.attitude_msg = Quaternion()
        self.esc_msg = String()
        self.battery_msg = Float64()
        self.image_pub = rospy.Publisher('/robomaster/image', Image, queue_size=1)
        self.imu_pub = rospy.Publisher('/robomaster/imu', Imu, queue_size=10)
        self.attitude_pub = rospy.Publisher('/robomaster/attitude', Quaternion, queue_size=10)
        self.esc_pub = rospy.Publisher('/robomaster/esc', String, queue_size=10)
        self.battery_pub = rospy.Publisher('/robomaster/battery', Float64, queue_size=10)
        # Publish imu data
        self.imu_thread = Thread(target=self.chassis.sub_imu(freq=1, callback=self.get_imu_msg))
        # Publish attitude data
        self.attitude_thread = Thread(target=self.chassis.sub_attitude(freq=1, callback=self.get_attitude_msg))
        # Publish esc data
        self.esc_thread = Thread(target=self.chassis.sub_esc(freq=1, callback=self.get_esc_msg))
        # Publish battery data
        self.battery_thread = Thread(target=self.battery.sub_battery_info(1, self.get_battery_msg, robot.Robot()))
        # Get status parameters
        self.status_thread = Thread(target=self.chassis.sub_status(freq=1, callback=self.get_status_params))


    def get_image_msg(self):
        """
        Gets image data from Robomaster's Python SDK,
        converts it from OpenCV image to a ROS message and
        then publishes converted message to the relevant topic
        """
        rospy.loginfo("Importing image data from robomaster")
        self.camera.start_video_stream(display=False)
        try:
            self.image_msg = CvBridge().cv2_to_imgmsg(self.camera.read_cv2_image(), "bgr8")
            self.image_pub.publish(self.image_msg)
        except CvBridgeError as e:
            print(e)
        self.camera.stop_video_stream()

    def get_imu_msg(self, imu_data):
        """
        Gets imu data from Robomaster's Python SDK,
        convers that to a ROS message and then
        publishes converted message to the relevant topic

        args:
        imu_data -- imu data that comes from robomaster's subscriber method
        """
        rospy.loginfo("Importing imu data from robomaster")
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_data
        self.imu_msg.linear_acceleration.x = acc_x
        self.imu_msg.linear_acceleration.y = acc_y
        self.imu_msg.linear_acceleration.z = acc_z
        self.imu_msg.orientation.x = gyro_x
        self.imu_msg.orientation.y = gyro_y
        self.imu_msg.orientation.z = gyro_z
        self.imu_pub.publish(self.imu_msg)

    def get_attitude_msg(self, attitude_info):
        """
        """
        rospy.loginfo("Importing attitude data from robomaster")
        yaw, pitch, roll = attitude_info
        self.attitude_msg.x = yaw
        self.attitude_msg.y = pitch
        self.attitude_msg.z = roll
        self.attitude_pub.publish(self.attitude_msg)

    def get_esc_msg(self, esc_info):
        """
        """
        rospy.loginfo("Importing esc data from robomaster")
        speed, angle, timestamp, state = esc_info
        self.esc_msg = speed + angle + timestamp + state
        #self.esc_msg.timestamp = timestamp
        #self.esc_msg.state = state
        self.esc_pub.publish(self.esc_msg)

    def get_battery_msg(self, batter_info, robot):
        """
        Gets battery data from Robomaster's Python SDK and
        convers it to ROS message and then
        publishes converted message to the relevant topic

        args:
        batter_info -- battery data that comes from robomaster's subscriber method
        robot -- robot instance from robomaster's robot class
        """
        rospy.loginfo("Importing battery data from robomaster")
        percent = batter_info
        self.battery_msg = percent
        self.battery_pub.publish(self.battery_msg)

    def get_status_params(self, status_info):
        """
        """
        rospy.loginfo("Setting status parameters for chassis from robomaster")
        static_flag, up_hill, down_hill, on_slope, pick_up, slip_flag, impact_x, impact_y, impact_z, roll_over, hill_static = status_info
        self.robomaster_static_flag = rospy.set_param('~robomaster/chassis/static_flag', static_flag)
        self.robomaster_up_hill = rospy.set_param('~robomaster/chassis/up_hill', up_hill)
        self.robomaster_down_hill = rospy.set_param('~robomaster/chassis/down_hill', down_hill)
        self.robomaster_on_slope = rospy.set_param('~robomaster/chassis/on_slope', on_slope)
        self.robomaster_pick_up = rospy.set_param('~robomaster/chassis/pick_up', pick_up)
        self.robomaster_slip_flag = rospy.set_param('~robomaster/chassis/slip_flag', slip_flag)
        self.robomaster_impact_x = rospy.set_param('~robomaster/chassis/impact_x', impact_x)
        self.robomaster_impact_y = rospy.set_param('~robomaster/chassis/impact_y', impact_y)
        self.robomaster_impact_z = rospy.set_param('~robomaster/chassis/impact_z', impact_z)
        self.robomaster_roll_over = rospy.set_param('~robomaster/chassis/roll_over', roll_over)
        self.robomaster_hill_static = rospy.set_param('~robomaster/chassis/hill_static', hill_static)

if __name__ == '__main__':
    # Initialize ROS node
    print('Initializing robomaster_talker node')
    rospy.init_node('robomaster_talker')
    print('Navigating to robomaster_talker node')
    rate = rospy.Rate(10) # 10hz
    # Take a loop until an interruption is made
    while not rospy.is_shutdown():
        robomaster_talker = RobomasterTalker()

        robomaster_talker.imu_thread.start()
        robomaster_talker.attitude_threa.start()
        robomaster_talker.esc_thread.start()
        robomaster_talker.battery_thread.start()
        robomaster_talker.status_thread.start()
        robomaster_talker.image_process.start()

    robomaster_talker.chassis.unsub_imu()
    robomaster_talker.chassis.unsub_attitude()
    robomaster_talker.chassis.unsub_esc()
    robomaster_talker.battery.unsub_battery_info()
    robomaster_talker.chassis.unsub_status()
    robomaster_talker.camera.stop_video_stream()
    robomaster_talker.close()