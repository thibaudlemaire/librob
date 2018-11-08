#!/usr/bin/env python

from librarian_msgs.srv import *
import rospy

def handle_db_comms(req):
	print "Returning code"	
	books =  "{ books: [{   title: Machine learning,author: Mitchell, Tom M. (Tom 		Michael),code: 006.31 MIT,floor: 1,available: true },{   title: Red seas under red 		skies,author: Lynch, Scott,code: 800 LYN floor: 5,available: true }]}"
	return db_requestResponse(books)

def db_comms_server():
	rospy.init_node('db_comms_server')
	s = rospy.Service('perform_database_request', db_request, handle_db_comms)
	print "Ready to convert."
	rospy.spin()

if __name__ == "__main__":
	db_comms_server()
