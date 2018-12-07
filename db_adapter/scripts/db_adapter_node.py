#!/usr/bin/env python
from librarian_msgs.srv import *
import rospy
import requests
import json
import re


class DbAdapter:
    def __init__(self):
        rospy.init_node('db_comms_server')
        self.s = rospy.Service('perform_database_request', db_request, self.handle_requests)

    def handle_requests(self, req):
        request = json.loads(req.request)
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

        if request.get('title', '') != '' and request.get('author', '') != '':
            payload['q'] = 'title,contains,%s,AND;author,contains,%s' % (request.get("title"), request.get("author"))
        elif request.get('author', '') != '':
            payload['q'] = 'author,contains,%s' % (request.get("author"))
        elif request.get('title', '') != '':
            payload['q'] = 'title,contains,%s' % (request.get("title"))

        resp = requests.get('https://api-eu.hosted.exlibrisgroup.com/primo/v1/search', params=payload)

        if resp.status_code != 200:
            raise Exception('GET /primo/v1/search/ {}'.format(resp.status_code))

        data = resp.json()

        for doc in data['docs']:
            book = dict()
            if doc.get("delivery", {}).get("bestlocation", {}).get("subLocation", '').startswith("L"):
                book['title'] = doc.get("pnx", {}).get("display", {}).get("title", [''])[0]
                book['author'] = doc.get("pnx", {}).get("sort", {}).get("author", [''])[0]
                book['code'] = doc.get("delivery", {}).get("bestlocation", {}).get("callNumber", [''])[1:-2]
                book['floor'] = int(doc.get("delivery", {}).get("bestlocation", {}).get("subLocation", 'Floor ?').split(" ")[1])
                book['available'] = (doc.get("delivery", {}).get("bestlocation", {}).get("availabilityStatus", False) == "available")
                links = doc.get('delivery', {}).get('link', {})
                thumbnail = next(link.get("linkURL", {}) for link in links if link.get("displayLabel", {} != ''))
                book['thumbnail'] = thumbnail
                if re.match('^[0-9]{1,3}(\.[0-9]{1,3}){0,2} [a-zA-Z]{3}$', book['code']):
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
