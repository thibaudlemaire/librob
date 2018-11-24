#!/usr/bin/env python
from librarian_msgs.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy
from locator import Locator


class LocatorNode():
	def __init__(self):
		rospy.init_node('locator_node')
		self.locator = Locator(rospy.get_param("~books_location"))
		self.service = rospy.Service('locate_book', book_locator, self.handle_requests)

	def handle_requests(self, req):
		print("Getting location request for " + str(req))
		location, orientation = self.locator.returnLocation(req.code)
		response = book_locatorResponse()
		response.floor = 5
		response.pose = Pose()
		response.pose.position = Point(location[0], location[1], location[2])
		response.pose.orientation = Quaternion(orientation[0], orientation[1],
											   orientation[2] , orientation[3])
		print(response)
		return response

	def spin_node(self):
		print("Locator node ready")
		rospy.spin()


if __name__ == "__main__":
	locator_node = LocatorNode()
	locator_node.spin_node()

