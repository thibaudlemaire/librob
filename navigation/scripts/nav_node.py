#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class Nav:
    def __init__(self):
        # self.nav_to_robot = tf.TransformBroadcaster()
        self.robot_to_laser = tf.TransformBroadcaster()
        # self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)

    # def broadcast_odom(self, pose_msg):
    #     position = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
    #     orientation = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
    #                    pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
    #     self.nav_to_robot.sendTransform(position,
    #                      orientation,
    #                      rospy.Time.now(),
    #                      "base_link",
    #                      "odom")


    def broadcast_static_transform(self):
        self.robot_to_laser.sendTransform((0.05, 0, 0.2),
                         (1, 0, 0, 0),
                         rospy.Time.now(),
                         "laser",
                         "base_link")

    def setup_node(self):
        rospy.init_node('simu_nav_node')
        # rospy.Subscriber("slam_out_pose", PoseStamped, self.broadcast_odom)
        print("Ready to transform")

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.broadcast_static_transform()
            rate.sleep()


if __name__ == '__main__':
    broadcaster = Nav()
    broadcaster.setup_node()
