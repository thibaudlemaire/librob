#!/usr/bin/env python
import rospy
import json
from librarian_msgs.msg import UI, UI_feedback
from librarian_msgs.srv import *
from geometry_msgs.msg import PoseStamped, Pose
from state_machine import StateMachine
from events import TimeOutEvent, GoalReachedEvent
import thread
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Behaviour:
    def __init__(self):
        rospy.init_node('behaviour_node', anonymous=True)
        self.ui_feedback_publisher = rospy.Publisher('ui_feedback', UI_feedback, queue_size=10)
        self.simple_goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.timer = None
        self.ac_goal = actionlib.SimpleActionClient('move_base', MoveBaseAction)
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
        self.state_machine = StateMachine(self)
        rospy.Subscriber("ui_command", UI, self.state_machine.on_event)

    def feedback_books(self, books_string):
        feedback_msg = UI_feedback()
        feedback_msg.type = UI_feedback.SEARCH_RESPONSE
        feedback_msg.payload = books_string
        self.ui_feedback_publisher.publish(feedback_msg)

    def feedback_message(self, msg, speak=True, title='', author=''):
        feedback_msg = UI_feedback()
        feedback_msg.type = UI_feedback.COMMUNICATION
        feedback_msg.payload = json.dumps({'type': msg, 'speak': speak, 'title': title, 'author': author})
        self.ui_feedback_publisher.publish(feedback_msg)

    def feedback_loading(self, state=True):
        feedback_msg = UI_feedback()
        feedback_msg.type = UI_feedback.LOADING
        feedback_msg.payload = json.dumps(state)
        self.ui_feedback_publisher.publish(feedback_msg)

    def wait_reached_goal(self):
        wait = self.ac_goal.wait_for_result()
        if wait and self.ac_goal.get_result():
            print("Goal reached !")
            self.state_machine.on_event(GoalReachedEvent())

    def new_goal(self, book_location):
        print("Waiting for moving base actionlib to be available")
        self.ac_goal.wait_for_server(rospy.Duration(5.0))
        print("Action goal server found")
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.pose = book_location.pose
        self.ac_goal.send_goal(goal_msg)
        thread.start_new_thread(self.wait_reached_goal, ())
        print("New goal set !")

    def timeout_callback(self, event):
        self.timer = None
        self.state_machine.on_event(TimeOutEvent())

    def set_timer(self, duration):
        if isinstance(self.timer, rospy.Timer):
            self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(duration), self.timeout_callback, True)

    def spin_node(self):
        print("Behaviour ready !")
        rospy.spin()


if __name__ == '__main__':
    behaviour = Behaviour()
    behaviour.spin_node()
