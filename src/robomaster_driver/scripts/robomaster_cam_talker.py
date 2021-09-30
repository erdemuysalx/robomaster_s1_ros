#!/usr/bin/env python

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from robomaster import robot


class RobomasterCameraTalker(robot.Robot):
    def __init__(self, cli=None):
        super().__init__(cli=cli)
        self.initialize(conn_type="sta")
        self.image_msg = Image()
        self.image_pub = rospy.Publisher('/robomaster/image', CompressedImage, queue_size=1)

    def get_image_msg(self):
        """
        Gets image data from Robomaster's Python SDK,
        converts it from OpenCV image to a ROS message and
        then publishes converted message to the relevant topic
        """
        rospy.loginfo("Importing image data from robomaster")
        self.camera.start_video_stream(display=False, resolution='720p')
        nb_frame = 0
        try:
            while True:
                self.image_msg = CvBridge().cv2_to_compressed_imgmsg(self.camera.read_cv2_image(), dst_format='jpg')
                info_str = ("#%s frame has been published" % nb_frame)
                rospy.loginfo(info_str)
                self.image_pub.publish(self.image_msg)
                nb_frame += 1
        except CvBridgeError as e:
            print(e)
        self.camera.stop_video_stream()



if __name__ == '__main__':
    # Initialize ROS node
    print('Initializing robomaster_camera_talker node')
    rospy.init_node('robomaster_camera_talker', anonymous=True)
    print('Navigating to robomaster_camera_talker node')
    rate = rospy.Rate(10) # 10hz
    # Take a loop until an interruption is made
    while not rospy.is_shutdown():
        # Create an instance from talker class
        robomaster_camera_talker = RobomasterCameraTalker()
        robomaster_camera_talker.get_image_msg()
        rate.sleep()
    robomaster_camera_talker.close()