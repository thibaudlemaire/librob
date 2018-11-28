#!/usr/bin/env python
import json
import rospy
from messages import Messages
from state import State
from librarian_msgs.msg import UI

station_floor = 5

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
    def __init__(self, node, floor_number, goal):
        super(MovingState, self).__init__(node, floor_number)
        self.global_goal = goal
        if goal.floor = self.floor:
        # Compute here the staps to achieve this goal (i.e : go to lift, jump in lift, go to book)
            self.current_goal = goal # For now we use the global goal as current goal
            self.substate = "TO_BOOK"
        else:
            self.current_goal = (xyz)#NEED LIFT DOOR COORDINATES
            self.substate = "TO_LIFT"
        self.node.new_goal(self.current_goal)
        self.node.feedback_message(Messages.FOLLOW_ME)

    def on_event(self, event):
        if isinstance(event, UI):
            self.node.feedback_message(Messages.BUSY)
        elif isinstance(event, "inserttype"): #NEED ARRIVE AT GOAL TYPE MESSAGE
            if self.substate == "TO_BOOK":
                return ReturnState(self.node, self.floor)
            elif self.substate == "TO_LIFT":
                self.substate = "WAIT"
            elif self.substate == "ENTER_LIFT":
                self.substate = "USING_LIFT"
        elif isinstance(event, "inserttype") #NEED LIFT DOOR OPEN MESSAGE
            if self.substate == "WAIT":
                self.current_goal = (xyz) #NEED LIFT COORDINATES
                self.node.new_goal(self.current_goal)
                self.substate = "ENTER_LIFT"
            elif self.substate = "USING_LIFT":
                self.current_goal = (self.global_goal)
                self.node.new_goal)self.current_goal)
                self.substate = "TO_BOOK"
        return self


class ReturnState(State):
    def __init__(self, node, floor_number):
        super(MovingState, self).__init__(node, floor_number)
        goal = (xyz) #NEED STATION COORDINATES
        self.global_goal = goal
        if goal.floor = self.floor:
        # Compute here the staps to achieve this goal (i.e : go to lift, jump in lift, go to book)
            self.current_goal = goal # For now we use the global goal as current goal
            self.substate = "TO_STATION"
        else:
            self.current_goal = (xyz)#NEED LIFT DOOR COORDINATES
            self.substate = "TO_LIFT"
        self.node.new_goal(self.current_goal)
        self.node.feedback_message(Messages.FOLLOW_ME)

    def on_event(self, event):
        if isinstance(event, UI):
            self.node.feedback_message(Messages.BUSY)
        elif isinstance(event, "inserttype"): #NEED ARRIVE AT GOAL TYPE MESSAGE
            if self.substate == "TO_STATION":
                return InitState(self.node, self.floor)
            elif self.substate == "TO_LIFT":
                self.substate = "WAIT"
            elif self.substate == "ENTER_LIFT":
                self.substate = "USING_LIFT"
        elif isinstance(event, "inserttype") #NEED LIFT DOOR OPEN MESSAGE
            if self.substate == "WAIT":
                self.current_goal = (xyz) #NEED LIFT COORDINATES
                self.node.new_goal(self.current_goal)
                self.substate = "ENTER_LIFT"
            elif self.substate = "USING_LIFT":
                self.current_goal = (self.global_goal)
                self.node.new_goal)self.current_goal)
                self.substate = "TO_STATION"
        return self
    


class StateMachine(object):
    def __init__(self, behaviour_node):
        self.node = behaviour_node
        self.floor = station_floor
        self.current_state = InitState(self.node,self.floor)

    def on_event(self, event):
        self.current_state = self.current_state.on_event(event)




