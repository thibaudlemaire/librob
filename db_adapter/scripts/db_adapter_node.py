#!/usr/bin/env python
from librarian_msgs.srv import *
import rospy
import requests
import json


class DbAdapter:
    def __init__(self):
        rospy.init_node('db_comms_server')
        self.s = rospy.Service('perform_database_request', db_request, self.handle_requests)

    def handle_requests(self, req):
        print("DB Adapter " + str(req))
        book_list = {'books': []}
        payload = {'vid': 'ICL_VU1',
                   'tab': 'all',
                   'scope': 'LRSCOP_44IMP',
                   'lang': 'eng',
                   'offset': '0',
                   'limit': '100',
                   'sort': 'rank',
                   'inst': '44IMP',
                   'multiFacets': 'facet_library,include,44IMP_CENTRAL_LIB|,|facet_rtype,include,books',
                   # facet_tlevel,include,available|,|
                   'mode': 'Basic',
                   'apikey': 'l7xx84a80e38fc0f482c9fdee23e3ede69f1',
                   'pcAvailability': 'true',
                   'conVoc': 'true'
                   }

        if 'title' in req.request and 'author' in req.request:
            payload['q'] = 'title,contains,%s,AND;author,contains,%s' % (req.request.get("title"), req.request.get("author"))
        elif 'title' not in req.request:
            payload['q'] = 'author,contains,%s' % (req.request.get("author"))
        elif 'author' not in req.request:
            payload['q'] = 'title,contains,%s' % (req.request.get("title"))

        resp = requests.get('https://api-eu.hosted.exlibrisgroup.com/primo/v1/search', params=payload)

        if resp.status_code != 200:
            raise Exception('GET /primo/v1/search/ {}'.format(resp.status_code))

        data = resp.json()

        for i in range(len(data["docs"])):
            book = dict()
            if data["docs"][i]["delivery"]["bestlocation"]["subLocation"].startswith("L"):
                book['title'] = data["docs"][i]["pnx"]["display"]["title"][0]
                book['author'] = data["docs"][i]["pnx"]["sort"]["author"][0]
                book['code'] = data["docs"][i]["delivery"]["bestlocation"]["callNumber"][1:-2]
                book['floor'] = int(data["docs"][i]["delivery"]["bestlocation"]["subLocation"].split(" ")[1])                    
                book['available'] = (data["docs"][i]["delivery"]["bestlocation"]["availabilityStatus"] == "available")
                book_list["books"].append(book)

        dumped_list = json.dumps(book_list)
        print(dumped_list)

        return db_requestResponse(dumped_list)

    def spin_node(self):
        print("DB Adapter node ready")
        rospy.spin()


if __name__ == "__main__":
    db_adapter = DbAdapter()
    db_adapter.spin_node()
