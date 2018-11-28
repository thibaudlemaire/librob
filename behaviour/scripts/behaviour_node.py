#!/usr/bin/env python
import rospy
import json
from librarian_msgs.msg import UI, UI_feedback
from librarian_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Pose
from state_machine import StateMachine
from messages import Messages


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
        self.state_machine = StateMachine()

    def process_ui(self, ui_msg):
        payload = json.loads(ui_msg.payload) if ui_msg != "" else {}

        if ui_msg.type == UI.SEARCH_REQUEST:
            self.feedback_message(Messages.SEARCHING + ' "' + payload['request'] + '"')
            self.feedback_loading()
            try:
                books = self.db_adapter_proxy(payload['request']).books
                if books == '{"books": []}':
                    self.feedback_message(Messages.NOT_FOUND + ' "' + payload['request'] + '"')
                else:
                    self.feedback_message(Messages.FOUND + ' "' + payload['request'] + '"')
                    self.feedback_books(books)
            except rospy.ServiceException:
                print("Error during db_adapter call !")
                self.feedback_message(Messages.DB_ERROR)
        elif ui_msg.type == UI.BOOK_CHOSEN:
            try:
                self.new_goal(self.locator_proxy(payload['chosen_code']))
                self.feedback_message(Messages.FOLLOW_ME)
            except rospy.ServiceException:
                print("Error during locator call !")
                self.feedback_message(Messages.LOCATOR_ERROR)
        elif ui_msg.type == UI.NOT_UNDERSTOOD:
            self.feedback_message(Messages.NOT_UNDERSTOOD)

    def feedback_books(self, books_string):
        feedback_msg = UI_feedback()
        feedback_msg.type = UI_feedback.SEARCH_RESPONSE
        feedback_msg.payload = books_string
        self.ui_feedback_publisher.publish(feedback_msg)

    def feedback_message(self, msg):
        feedback_msg = UI_feedback()
        feedback_msg.type = UI_feedback.COMMUNICATION
        feedback_msg.payload = json.dumps({'message': msg})
        self.ui_feedback_publisher.publish(feedback_msg)

    def feedback_loading(self, state=True):
        feedback_msg = UI_feedback()
        feedback_msg.type = UI_feedback.LOADING
        feedback_msg.payload = json.dumps(state)
        self.ui_feedback_publisher.publish(feedback_msg)

    def new_goal(self, book_location):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose = book_location.pose
        self.simple_goal_publisher.publish(goal_msg)
        print("New goal set !")

    def start_node(self):
        rospy.init_node('behaviour_node', anonymous=True)
        rospy.Subscriber("ui_command", UI, self.process_ui)
        print("Behaviour ready !")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.state_machine.process_state_machine()
            rate.sleep()


if __name__ == '__main__':
    behaviour = Behaviour()
    behaviour.start_node()
