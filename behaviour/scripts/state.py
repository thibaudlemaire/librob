from geometry_msgs.msg import Pose


class State(object):
    def __init__(self, behaviour_node, current_floor):
        self.node = behaviour_node
        self.floor = current_floor

    def on_event(self, event):
        pass


class Goal(object):
    def __init__(self, location = None):
        pass

    def set_pose(self, pose):
        self.pose = pose

    def set_floor(self, floor):
        self.floor = floor

    def set_from_location(self, location):
        pass