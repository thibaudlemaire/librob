#!/usr/bin/env python

from librarian_msgs.srv import *
import rospy
import requests
import json


def handle_db_comms(req):
	print "Returning code"
	
	book_list = {'books':{'title':'', 'code':'', 'author':'', 'availability':'', 'floor':''}}
	
	payload = {'vid' : 'ICL_VU1', 'tab' : 'all', 'scope' : 'LRSCOP_44IMP', 'q' : 'title,contains,%s'%(req.request), 'lang': 'eng', 'offset' : '0', 'limit' : '1', 'sort' : 'rank', 'pcAvailability' : 'true', 'conVoc' : 'false', 'inst' : '44IMP', 'mfcet' : 'rtype,include,books,1', 'mfacet' : 'tlevel,include,available,2', 'mfacet' : 'library,include,44IMP_CENTRAL_LIB,3', 'lang' : 'en_US', 'mode' : 'Basic', 'apikey' : 'l7xx84a80e38fc0f482c9fdee23e3ede69f1'}
	

	resp = requests.get('https://api-eu.hosted.exlibrisgroup.com/primo/v1/search', params = payload)
	
	if resp.status_code != 200:
		raise ApiError('GET /primo/v1/search/ {}' .format(resp.status_code))	
	

	books =  "{ books: [{   title: Machine learning,author: Mitchell, Tom M. (Tom 		Michael),code: 006.31 MIT,floor: 1,available: true },{   title: Red seas under red 		skies,author: Lynch, Scott,code: 800 LYN floor: 5,available: true }]}"	
	
	data = resp.json()	
	
	book_list['books']['code'] = data["docs"][0]["delivery"]["bestlocation"]["callNumber"]
	book_list['books']['title'] = data["docs"][0]["pnx"]["display"]["title"]
	book_list['books']['author'] = data["docs"][0]["pnx"]["display"]["creator"]
	book_list['books']['availability'] = data["docs"][0]["delivery"]["bestlocation"]["availabilityStatus"]
	book_list['books']['floor'] = data["docs"][0]["delivery"]["bestlocation"]["subLocation"]
	
	json_list = json.dumps(book_list, indent = 4, sort_keys = True)
	
	print json_list
	return db_requestResponse(books)

def db_comms_server():
	rospy.init_node('db_comms_server')
	s = rospy.Service('perform_database_request', db_request, handle_db_comms)
	print "Ready to convert."
	rospy.spin()

if __name__ == "__main__":
	db_comms_server()
