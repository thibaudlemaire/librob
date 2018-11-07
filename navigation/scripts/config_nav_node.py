#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class RobotTfBroadcaster:
    def __init__(self):
        self.nav_to_robot = tf.TransformBroadcaster()
        self.robot_to_laser = tf.TransformBroadcaster()

    def broadcast_odom(self, pose_msg):
        position = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
        orientation = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                       pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
        self.nav_to_robot.sendTransform(position,
                         orientation,
                         rospy.Time.now(),
                         "base_link",
                         "odom")

    def broadcast_static_transform(self):
        self.robot_to_laser.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "laser",
                         "base_link")

    def start_broadcast(self):
        rospy.init_node('config_nav_node')
        rospy.Subscriber("slam_out_pose", PoseStamped, self.broadcast_odom)
        print("Ready to transform")

        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            self.broadcast_static_transform()
            rate.sleep()


if __name__ == '__main__':
    broadcaster = RobotTfBroadcaster()
    broadcaster.start_broadcast()