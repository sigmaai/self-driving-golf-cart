
import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive


class SelfDrivingSystemBridge():

    def __init__(self):

        rospy.init_node("self_driving_system_bridge_node")

        self.steering_cmd = 0
        self.speed_cmd = 0

        rospy.Subscriber('/vehicle/dbw/steering_cmds/', Float32, callback=self.steering_cmd_callback)
        rospy.Subscriber('/vehicle/dbw/cruise_cmds', Float32, callback=self.speed_cmd_callback)

        ackermann_pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive)

        rate = rospy.Rate(30)

        rospy.loginfo("self-driving system bridge started")

        while not rospy.is_shutdown():

            ackermann_msg = AckermannDrive()
            ackermann_msg.steering_angle = self.steering_cmd
            ackermann_msg.speed = self.speed_cmd

            ackermann_pub.publish(ackermann_msg)

            rate.sleep()

    def steering_cmd_callback(self, data):

        self.steering_cmd = data

    def speed_cmd_callback(self, data):

        self.speed_cmd = data


if __name__ == "__main__":

    try:
        SelfDrivingSystemBridge()
    except rospy.ROSInitException:
        pass