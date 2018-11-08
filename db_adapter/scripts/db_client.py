#!/usr/bin/env python

import sys
import rospy
from librarian_msgs.srv import *

def db_comms_client(request):
	rospy.wait_for_service('perform_database_request')
	try:
		perform_database_request = rospy.ServiceProxy('perform_database_request', db_request)
		resp1 = perform_database_request(request)
		return resp1.books
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e
def usage():
	return "%s name"%sys.argv[0]

if __name__ == "__main__":
	if len(sys.argv) == 2:
		request = str(sys.argv[1])
	else:
		print usage
		sys.exit(1)
	print "Requesting code for: %s"%(request)
	print "code for %s is %s"%(request,db_comms_client(request))
