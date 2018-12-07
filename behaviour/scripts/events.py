class Event(object):
    def __init__(self, event_type):
        self.event_type = event_type


class TimeOutEvent(Event):
    def __init__(self):
        super(TimeOutEvent, self).__init__('TIME_OUT_EVENT')