#!/usr/bin/env python

from librarian_msgs.srv import *
import rospy
import requests
import json


def handle_db_comms(req):
	print "Returning code"
	
	book_list = {'books': []}
	
	payload = {'vid' : 'ICL_VU1', 'tab' : 'all', 'scope' : 'LRSCOP_44IMP', 'q' : 'title,contains,%s'%(req.request), 'lang': 'eng', 'offset' : '0', 'limit' : '5', 'sort' : 'rank', 'inst' : '44IMP', 'mfacet' : 'tlevel,include,available,2', 'mfacet' : 'library,include,44IMP_CENTRAL_LIB,3', 'mfacet' : 'rtype,include,books,1', 'lang' : 'en_US', 'mode' : 'Basic', 'apikey' : 'l7xx84a80e38fc0f482c9fdee23e3ede69f1', 'pcAvailability' : 'true',  'conVoc' : 'true'}
	

	resp = requests.get('https://api-eu.hosted.exlibrisgroup.com/primo/v1/search', params = payload)
	
	if resp.status_code != 200:
		raise ApiError('GET /primo/v1/search/ {}' .format(resp.status_code))	
	
	
	data = resp.json()

	for i in range(len(data["docs"])):
		book = dict()
		if (data["docs"][i]["pnx"]["display"]["type"][0] == "book" and data["docs"][i]["delivery"]["bestlocation"]["subLocation"].startswith("L") ):
	
			book['code'] = data["docs"][i]["delivery"]["bestlocation"]["callNumber"][1:-2]
			book['title'] = data["docs"][i]["pnx"]["display"]["title"][0]
			book['author'] = data["docs"][i]["pnx"]["sort"]["author"][0]
			book['available'] = (data["docs"][i]["delivery"]["bestlocation"]["availabilityStatus"] == "available")
			book['floor'] = int(data["docs"][i]["delivery"]["bestlocation"]["subLocation"].split(" ")[1])
			book_list["books"].append(book)
		#print(i)
	
	json_list = json.dumps(book_list, indent = 4, sort_keys = True)
	
	#print json_list
	#print(resp.url)
	return db_requestResponse(json.dumps(book_list))
	print("Returning code")
	#books = {'books': [
		{
			'title': 'Machine learning',
			'author': 'Mitchell, Tom M. (Tom Michael)',
			'code': '006.31 MIT',
			'floor': 1,
			'available': True
		},{
			'title': 'Red seas under red skies',
			'author': 'Lynch, Scott', 'code': '800 LYN',
			'floor': 5,
			'available': True
		}
	]}
	#return db_requestResponse(json.dumps(books))

def db_comms_server():
	rospy.init_node('db_comms_server')
	s = rospy.Service('perform_database_request', db_request, handle_db_comms)
	print("Ready to convert.")
	rospy.spin()


if __name__ == "__main__":
	db_comms_server()
