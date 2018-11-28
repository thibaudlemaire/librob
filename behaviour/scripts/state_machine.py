#!/usr/bin/env python


class StateMachine:
    # States :
    IDLE_STATE = 1
    USER_INTERACT_STATE = 2
    MOVING_STATE = 3
    SHOWING_BOOK_STATE = 4

    def __init__(self):
        self.current_state = StateMachine.IDLE_STATE

    def process_state_machine(self):
        if self.current_state == StateMachine.IDLE_STATE:
            pass
        elif self.current_state == StateMachine.USER_INTERACT_STATE:
            pass
        elif self.current_state == StateMachine.MOVING_STATE:
            pass
        elif self.current_state == StateMachine.SHOWING_BOOK_STATE:
            pass




