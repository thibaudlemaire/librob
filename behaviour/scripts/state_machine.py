#!/usr/bin/env python
import json
import rospy
from messages import Messages
from state import State
from librarian_msgs.msg import UI

LIFT_GOAL = None
STATION_GOAL = None
DOOR_OPEN_EVENT = object
GOAL_REACHED_EVENT = object


class InitState(State):
    def on_event(self, event):
        if isinstance(event, UI):
            ui_msg = event
            if ui_msg.type == UI.BOOK_CHOSEN:
                payload = json.loads(ui_msg.payload) if ui_msg != "" else {}
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
        payload = json.loads(ui_msg.payload) if ui_msg != "" else {}

        if ui_msg.type == UI.SEARCH_REQUEST:
            self.node.feedback_message(Messages.SEARCHING + ' "' + payload['request'] + '"')
            self.node.feedback_loading()
            try:
                books = self.node.db_adapter_proxy(payload['request']).books
                if books == '{"books": []}':
                    self.node.feedback_message(Messages.NOT_FOUND + ' "' + payload['request'] + '"')
                else:
                    self.node.feedback_message(Messages.FOUND + ' "' + payload['request'] + '"')
                    self.node.feedback_books(books)
            except rospy.ServiceException:
                print("Error during db_adapter call !")
                self.node.feedback_message(Messages.DB_ERROR)
        elif ui_msg.type == UI.NOT_UNDERSTOOD:
            self.node.feedback_message(Messages.NOT_UNDERSTOOD)


class MovingState(State):
    TO_BOOK = 1
    TO_LIFT = 2
    WAIT = 3
    ENTER_LIFT = 4
    USING_LIFT = 5

    def __init__(self, node, floor_number, goal):
        super(MovingState, self).__init__(node, floor_number)
        self.global_goal = goal
        if goal.floor == self.floor:
            self.current_goal = goal
            self.substate = MovingState.TO_BOOK
        else:
            self.current_goal = LIFT_GOAL
            self.substate = MovingState.TO_LIFT
        self.node.new_goal(self.current_goal)
        self.node.feedback_message(Messages.FOLLOW_ME)

    def on_event(self, event):
        if isinstance(event, UI):
            self.node.feedback_message(Messages.BUSY)
        elif isinstance(event, GOAL_REACHED_EVENT):
            if self.substate == MovingState.TO_BOOK:
                return ReturnState(self.node, self.floor)
            elif self.substate == MovingState.TO_LIFT:
                self.substate = MovingState.WAIT
            elif self.substate == MovingState.ENTER_LIFT:
                self.substate = MovingState.ENTER_LIFT
        elif isinstance(event, DOOR_OPEN_EVENT):
            if self.substate == MovingState.WAIT:
                self.current_goal = LIFT_GOAL
                self.node.new_goal(self.current_goal)
                self.substate = MovingState.ENTER_LIFT
            elif self.substate == MovingState.USING_LIFT:
                self.current_goal = self.global_goal
                self.node.new_goal(self.current_goal)
                self.substate = MovingState.TO_BOOK
        return self


class ReturnState(State):
    def __init__(self, node, floor_number):
        super(ReturnState, self).__init__(node, floor_number)
        self.global_goal = STATION_GOAL
        if self.global_goal.floor == self.floor:
            self.current_goal = STATION_GOAL
            self.substate = "TO_STATION"
        else:
            self.current_goal = LIFT_GOAL
            self.substate = "TO_LIFT"
        self.node.new_goal(self.current_goal)
        self.node.feedback_message(Messages.FOLLOW_ME)

    def on_event(self, event):
        if isinstance(event, UI):
            self.node.feedback_message(Messages.BUSY)
        elif isinstance(event, GOAL_REACHED_EVENT):
            if self.substate == "TO_STATION":
                return InitState(self.node, self.floor)
            elif self.substate == "TO_LIFT":
                self.substate = "WAIT"
            elif self.substate == "ENTER_LIFT":
                self.substate = "USING_LIFT"
        elif isinstance(event, DOOR_OPEN_EVENT):
            if self.substate == "WAIT":
                self.current_goal = LIFT_GOAL
                self.node.new_goal(self.current_goal)
                self.substate = "ENTER_LIFT"
            elif self.substate == "USING_LIFT":
                self.current_goal = self.global_goal
                self.node.new_goal(self.current_goal)
                self.substate = "TO_STATION"
        return self


class StateMachine(object):
    def __init__(self, behaviour_node):
        self.node = behaviour_node
        self.current_state = InitState(self.node, STATION_GOAL.floor)

    def on_event(self, event):
        self.current_state = self.current_state.on_event(event)
