class Event(object):
    def __init__(self, event_type):
        self.event_type = event_type


class TimeOutEvent(Event):
    def __init__(self):
        super(TimeOutEvent, self).__init__('TIME_OUT_EVENT')


class FrontClearEvent(Event):
    def __init__(self):
        super(FrontClearEvent, self).__init__('FRONT_CLEAR_EVENT')


class FrontOccupiedEvent(Event):
    def __init__(self):
        super(FrontOccupiedEvent, self).__init__('FRONT_OCCUPIED_EVENT')


class GoalReachedEvent(Event):
    def __init__(self):
        super(GoalReachedEvent, self).__init__('GOAL_REACHED_EVENT')
