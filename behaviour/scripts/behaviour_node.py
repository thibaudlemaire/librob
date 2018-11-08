#!/usr/bin/env python
import rospy
import json
from librarian_msgs.msg import UI, UI_feedback
from librarian_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Pose


class Behaviour:
    def __init__(self):
        self.ui_feedback_publisher = rospy.Publisher('ui_feedback', UI_feedback, queue_size=10)
        self.simple_goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        # Creation of service proxies
        print("Waiting for db_adapter and locator services...")
        rospy.wait_for_service('perform_database_request', timeout=10)
        rospy.wait_for_service('locate_book', timeout=10)
        print("Services found, setting up proxies")
        try:
            self.db_adapter_proxy = rospy.ServiceProxy('perform_database_request', db_request)
            self.locator_proxy = rospy.ServiceProxy('locate_book', book_locator)
        except rospy.ServiceException:
            print("Service proxy creation failed !")

    def process_ui(self, ui_msg):
        payload = json.loads(ui_msg.payload)
        if ui_msg.type == UI.SEARCH_REQUEST:
            feedback_msg = UI_feedback()
            feedback_msg.type = UI_feedback.SEARCH_RESPONSE
            try:
                feedback_msg.payload = self.db_adapter_proxy(payload['request'])
            except rospy.ServiceException:
                print("Error during db_adapter call !")
            self.ui_feedback_publisher.publish(feedback_msg)
        elif ui_msg.type == UI.BOOK_CHOSEN:
            feedback_msg = UI_feedback()
            feedback_msg.type = UI_feedback.LOADING
            self.ui_feedback_publisher.publish(feedback_msg)
            pose = Pose()
            try:
                (pose, floor) = self.locator_proxy(payload['chosen_code'])
            except rospy.ServiceException:
                print("Error during locator call !")
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.pose = pose
            print("New goal set !")

    def start_node(self):
        rospy.init_node('behaviour_node', anonymous=True)
        rospy.Subscriber("ui_command", UI, self.process_ui)

        print("Behaviour ready !")
        rospy.spin()


if __name__ == '__main__':
    behaviour = Behaviour()
    behaviour.start_node()
