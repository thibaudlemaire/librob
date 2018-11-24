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
                   'q': 'title,contains,%s' % (req.request),
                   'lang': 'eng',
                   'offset': '0',
                   'limit': '5',
                   'sort': 'rank',
                   'inst': '44IMP',
                   'mfacet': 'tlevel,include,available,2;library,include,44IMP_CENTRAL_LIB,3;rtype,include,books,1',
                   'mode': 'Basic',
                   'apikey': 'l7xx84a80e38fc0f482c9fdee23e3ede69f1',
                   'pcAvailability': 'true',
                   'conVoc': 'true'
                   }

        resp = requests.get('https://api-eu.hosted.exlibrisgroup.com/primo/v1/search', params=payload)

        if resp.status_code != 200:
            raise ApiError('GET /primo/v1/search/ {}'.format(resp.status_code))

        data = resp.json()

        for i in range(len(data["docs"])):
            book = dict()
            if (data["docs"][i]["pnx"]["display"]["type"][0] == "book" and data["docs"][i]["delivery"]["bestlocation"][
                "subLocation"].startswith("L")):
                book['code'] = data["docs"][i]["delivery"]["bestlocation"]["callNumber"][1:-2]
                book['title'] = data["docs"][i]["pnx"]["display"]["title"][0]
                book['author'] = data["docs"][i]["pnx"]["sort"]["author"][0]
                book['available'] = (data["docs"][i]["delivery"]["bestlocation"]["availabilityStatus"] == "available")
                book['floor'] = int(data["docs"][i]["delivery"]["bestlocation"]["subLocation"].split(" ")[1])
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
