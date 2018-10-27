#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Adapter:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def callback(self, joy_msg):
        velocity = Twist()
        velocity.linear.x = 0.1 * joy_msg.axes[1]
        velocity.angular.z = 0.1 * joy_msg.axes[0]
        self.cmd_vel_pub.publish(velocity)

    def listener(self):
        rospy.init_node('joy_to_p2os_adapter', anonymous=True)
        rospy.Subscriber("joy", Joy, self.callback)
        rospy.loginfo("Listener ready")
        rospy.spin()


if __name__ == '__main__':
    adapter = Adapter()
    adapter.listener()
