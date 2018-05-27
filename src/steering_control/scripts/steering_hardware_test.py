

import pandas as pd

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError


class HardwareTest(object):

    def __init__(self):

        rospy.init_node('steering_hardware_test')
        rospy.Subscriber('/cv_camera_node/image_raw', Image, callback=self.image_update_callback, queue_size=5)

        self.current_frame = None
        self.bridge = CvBridge()

        # TODO: load steering angle csv

        data_path = ""
        self.values = pd.read_csv(data_path).values
        count = 0

        self.scale_factor = rospy.get_param("/steering_controller/scale_factor")
        rospy.loginfo('loaded scaling factor: %s', self.scale_factor)

        steering_pub = rospy.Publisher('/vehicle/dbw/steering_cmds/', Float32, queue_size=5)
        rate = rospy.Rate(15)

        while not rospy.is_shutdown() and self.current_frame is not None:
            angle = self.values[count][2]
            angle = angle * int(self.scale_factor)
            steering_pub.publish(angle)
            count += 1
            rate.sleep()

    def image_update_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        self.current_frame = cv_image


if __name__ == "__main__":

    try:
        HardwareTest()
    except rospy.ROSInterruptException:
        pass

