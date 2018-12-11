#!/usr/bin/env python
import json
import rospy
from messages import Messages
from state import State, Goal
from librarian_msgs.msg import UI
from events import *

LIFT_GOAL = Goal()
STATION_GOAL = Goal()


class InitState(State):
    def on_event(self, event):
        if isinstance(event, UI):
            ui_msg = event
            if ui_msg.type == UI.BOOK_CHOSEN:
                payload = json.loads(ui_msg.payload) if ui_msg.payload != "" else {}
                try:
                    goal = self.node.locator_proxy(payload['chosen_code'])
                    return MovingState(self.node, self.floor, goal)
                except rospy.ServiceException:
                    print("Error during locator call !")
                    self.node.feedback_message(Messages.LOCATOR_ERROR)
            else:
                self.process_ui(ui_msg)
        return self

    def process_ui(self, ui_msg):
        payload = json.loads(ui_msg.payload) if ui_msg.payload != "" else {}
        if ui_msg.type == UI.SEARCH_REQUEST:
            request = payload['request']
            print(request)
            title = request.get('title', '')
            author = request.get('author', '')
            self.node.feedback_message(Messages.SEARCHING, False)
            self.node.feedback_loading()
            try:
                books = self.node.db_adapter_proxy(json.dumps(payload['request'])).books
                if books == '{"books": []}':
                    self.node.feedback_message(Messages.NOT_FOUND, True, title, author)
                else:
                    self.node.feedback_message(Messages.FOUND, True, title, author)
                    self.node.feedback_books(books)
            except rospy.ServiceException:
                print("Error during db_adapter call !")
                self.node.feedback_message(Messages.DB_ERROR)
        elif ui_msg.type == UI.NOT_UNDERSTOOD:
            self.node.feedback_message(Messages.NOT_UNDERSTOOD)
        elif ui_msg.type == UI.SPEECH_TRIGGER:
            self.node.feedback_message(Messages.HOW_TO_TALK)


class MovingState(State):
    TO_BOOK = 1
    TO_LIFT = 2
    WAIT = 3
    ENTER_LIFT = 4
    USING_LIFT = 5

    def __init__(self, node, current_floor, goal):
        super(MovingState, self).__init__(node, current_floor)
        self.global_goal = goal
        if goal.floor == self.floor:
            self.current_goal = goal
            self.substate = MovingState.TO_BOOK
        else:
            self.current_goal = LIFT_GOAL
            self.substate = MovingState.TO_LIFT
        self.node.new_goal(self.current_goal)
        self.node.feedback_message(Messages.FOLLOW_ME)
        self.node.set_timer(60)

    def on_event(self, event):
        if isinstance(event, UI):
            self.node.feedback_message(Messages.BUSY, True)
        elif isinstance(event, GoalReachedEvent):
            if self.substate == MovingState.TO_BOOK:
                if global_goal == STATION_GOAL:
                    return InitState
                else:
                    return FinalState(self.node, self.floor)
            elif self.substate == MovingState.TO_LIFT:
                self.substate = MovingState.WAIT
            elif self.substate == MovingState.ENTER_LIFT:
                self.substate = MovingState.ENTER_LIFT
        elif isinstance(event, FrontClearEvent):
            if self.substate == MovingState.WAIT:
                self.current_goal = LIFT_GOAL
                self.node.new_goal(self.current_goal)
                self.substate = MovingState.ENTER_LIFT
            elif self.substate == MovingState.USING_LIFT:
                self.current_goal = self.global_goal
                self.node.new_goal(self.current_goal)
                self.substate = MovingState.TO_BOOK
        elif isinstance(event, TimeOutEvent):
            self.node.feedback_message(Messages.TIME_OUT, True)
            return InitState(self.node, self.floor)
        return self


class FinalState(State):
    def __init__(self, node, current_floor):
        super(FinalState, self).__init__(node, current_floor)
        self.node.set_timer(5)

    def on_event(self, event):
        if isinstance(event, TimeOutEvent):
            self.node.feedback_message(Messages.TIME_OUT, True)
            return MovingState(self.node, current_floor, STATION_GOAL)


class StateMachine(object):
    def __init__(self, behaviour_node):
        self.node = behaviour_node
        self.current_state = InitState(self.node, 1)

    def on_event(self, event):
        self.current_state = self.current_state.on_event(event)
