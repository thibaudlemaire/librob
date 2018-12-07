#!/usr/bin/env python
import json
import rospy
from messages import Messages
from state import State
from librarian_msgs.msg import UI


class InitState(State):
    def on_event(self, event):
        if isinstance(event, UI):
            ui_msg = event
            if ui_msg.type == UI.BOOK_CHOSEN:
                payload = json.loads(ui_msg.payload) if ui_msg.payload != "" else {}
                try:
                    goal = self.node.locator_proxy(payload['chosen_code'])
                    # return MovingState(self.node, goal)
                    MovingState(self.node, goal) # For the demo, we do not change state
                except rospy.ServiceException:
                    print("Error during locator call !")
                    self.node.feedback_message(Messages.LOCATOR_ERROR)
            else:
                self.process_ui(ui_msg)
        return self

    def process_ui(self, ui_msg):
        payload = json.loads(ui_msg.payload) if ui_msg.payload != "" else {}
        if ui_msg.type == UI.SEARCH_REQUEST:
            self.node.feedback_message(Messages.SEARCHING + ' "' + payload['request'].get('title') + '"', False)
            self.node.feedback_loading()
            try:
                books = self.node.db_adapter_proxy(json.dumps(payload['request'])).books
                if books == '{"books": []}':
                    self.node.feedback_message(Messages.NOT_FOUND + ' "' + payload['request'].get('title') + '"')
                else:
                    self.node.feedback_message(Messages.FOUND + ' "' + payload['request'].get('title') + '"')
                    self.node.feedback_books(books)
            except rospy.ServiceException:
                print("Error during db_adapter call !")
                self.node.feedback_message(Messages.DB_ERROR)
        elif ui_msg.type == UI.NOT_UNDERSTOOD:
            self.node.feedback_message(Messages.NOT_UNDERSTOOD)
        elif ui_msg.type == UI.SPEECH_TRIGGER:
            self.node.feedback_message(Messages.HOW_TO_TALK)


class MovingState(State):
    def __init__(self, node, goal):
        super(MovingState, self).__init__(node)
        self.global_goal = goal
        # Compute here the staps to achieve this goal (i.e : go to lift, jump in lift, go to book)
        self.current_goal = goal # For now we use the global goal as current goal
        self.node.new_goal(self.current_goal)
        self.node.feedback_message(Messages.FOLLOW_ME)

    def on_event(self, event):
        if isinstance(event, UI):
            self.node.feedback_message(Messages.BUSY)
        return self


class FinalState(State):
    def on_event(self, event):
        return self


class StateMachine(object):
    def __init__(self, behaviour_node):
        self.node = behaviour_node
        self.current_state = InitState(self.node)

    def on_event(self, event):
        self.current_state = self.current_state.on_event(event)




