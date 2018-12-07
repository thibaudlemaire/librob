from geometry_msgs.msg import Pose
from librarian_msgs.srv import *


class State(object):
    def __init__(self, behaviour_node, current_floor):
        self.node = behaviour_node
        self.floor = current_floor

    def on_event(self, event):
        pass


class Goal(object):
    def __init__(self, pose = None, floor = 0, location = None):
        if isinstance(location, db_requestResponse):
            self.pose = location.pose
            self.floor = location.floor
        elif isinstance(pose, Pose):
            self.pose = pose
            self.floor = floor
        else:
            self.pose = Pose()
            self.pose.position.x = 0
            self.pose.position.y = 0
            self.pose.position.z = 0
            self.pose.orientation.x = 0
            self.pose.orientation.y = 0
            self.pose.orientation.z = 0
            self.pose.orientation.w = 0
            self.floor = 0

    def set_pose(self, pose):
        self.pose = pose
        return self

    def set_floor(self, floor):
        self.floor = floor
        return self
