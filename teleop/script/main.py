#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Adapter:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cur_velocity = Twist()

    def callback(self, joy_msg):
        self.cur_velocity.linear.x = rospy.get_param("~coef", 0.1) * joy_msg.axes[1]
        self.cur_velocity.angular.z = rospy.get_param("~coef", 0.1) * joy_msg.axes[3]
        self.cmd_vel_pub.publish(self.cur_velocity)

    def listener(self):
        rospy.init_node('joy_to_p2os_adapter', anonymous=True)
        rospy.Subscriber("joy", Joy, self.callback)
        rospy.loginfo("Listener ready")
        rate = rospy.Rate(5)  # 5hz
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cur_velocity)
            rate.sleep()


if __name__ == '__main__':
    adapter = Adapter()
    adapter.listener()
