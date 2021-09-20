#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robomaster import robot


class RobomasterTalker():
    def __init__(self):
        self.robot = robot.Robot()  # Robomaster robot instance
        self.robot.initialize(conn_type="sta")
        self.chassis = self.robot.chassis
        self.camera = self.robot.camera
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher('/robomaster/image', Image, queue_size=1)
        self.imu_pub = rospy.Publisher('/robomaster/imu', Float64, queue_size=10)
        self.imu_data_handler = self.chassis.sub_imu(freq=5, callback=self.set_imu_data)
        self.rate = rospy.Rate(10)

    def get_image_msg(self):
        """ 
        Converts image data from OpenCV image to ROS message

        returns:

        image_msg -- ROS image message for image data 
        """
        self.camera.start_video_stream(display=False)
        for i in range(0, 200):
            img = self.camera.read_cv2_image()
            try:
                img = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.camera.stop_video_stream()
                return img
            except CvBridgeError as e:
                print(e)

    def set_imu_data(self, imu_data):
        """ 
        Get imu data from Robomaster Python SDK
        
        args:
        imu_data -- imu data

        returns:
        acc_x -- accelaration on x axis
        acc_y -- accelaration on y axis
        acc_z -- accelaration on z axis
        gyro_x -- gyro on x axis
        gyro_y -- gyro on y axis
        gyro_z -- gyro on z axis
        """
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_data,
        return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

    def get_imu_msg(self):
        """ 
        Converts imu data from Robomaster SDK to ROS message

        returns:

        imu_msg -- ROS message for imu data 
        """
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.set_imu_data()
        imu_msg = String()
        imu_msg = "acc_x: {}, acc_y: {}, acc_y: {}, gyro_x: {}, gyro_y: {}, gyro_z: {}".format(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
        return imu_msg


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('robomaster_talker')
        # Create an instance from talker class
        robomaster_talker = RobomasterTalker()
        # Unless there is an interruption
        while not rospy.is_shutdown():
            # Publish image_msg to image topic
            robomaster_talker.img_pub.publish(robomaster_talker.get_image_msg())
            # Publish imu_msg to imu topic
            robomaster_talker.imu_pub.publish(robomaster_talker.get_imu_msg())

            # Set publish frequency rate
            robomaster_talker.rate.sleep()
            
    except rospy.ROSInterruptException:
        # Delete Robomaster robot instance
        robomaster_talker.robot.close()