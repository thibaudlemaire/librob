#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class SimuNav:
    def __init__(self):
        self.nav_to_robot = tf.TransformBroadcaster()
        self.robot_to_laser = tf.TransformBroadcaster()
        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)

    def broadcast_odom(self, pose_msg):
        position = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
        orientation = (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                       pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)
        self.nav_to_robot.sendTransform(position,
                         orientation,
                         rospy.Time.now(),
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = pose_msg.pose.position.x
        odom.pose.pose.position.y = pose_msg.pose.position.y
        odom.pose.pose.position.z = pose_msg.pose.position.z

        odom.pose.pose.orientation.x = pose_msg.pose.orientation.x
        odom.pose.pose.orientation.y = pose_msg.pose.orientation.y
        odom.pose.pose.orientation.z = pose_msg.pose.orientation.z
        odom.pose.pose.orientation.w = pose_msg.pose.orientation.w

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_publisher.publish(odom)

    def broadcast_static_transform(self):
        self.robot_to_laser.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "laser",
                         "base_link")

    def setup_node(self):
        rospy.init_node('simu_nav_node')
        rospy.Subscriber("slam_out_pose", PoseStamped, self.broadcast_odom)
        print("Ready to transform")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.broadcast_static_transform()
            rate.sleep()


if __name__ == '__main__':
    broadcaster = SimuNav()
    broadcaster.setup_node()