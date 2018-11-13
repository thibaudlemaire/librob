#!/usr/bin/env python

from librarian_msgs.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy


def handle_requests(req):
	response = book_locatorResponse()
	response.floor = 1
	response.pose = Pose()
	response.pose.position = Point(100, 104, 0)
	response.pose.orientation = Quaternion(0,0,0,1)
	return response


def locator_node():
	rospy.init_node('locator_node')
	s = rospy.Service('locate_book', book_locator, handle_requests)
	print("Locator node ready")
	rospy.spin()


if __name__ == "__main__":
	locator_node()

