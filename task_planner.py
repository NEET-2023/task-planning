from generator import *
from evaluator import *
import rospy
from geometry_msgs.msg import Point

starting = 0
flying = 1
dropping = 2
picking = 3
returning = 5

class TaskPlanner:
    
    def __init__(self, eval: Evaluator):
        self.state = starting
        self.placements = []
        self.current_sensor = 0
        self.eval = eval
        variables, height, width = eval.get_dims()
        self.map_pub = rospy.Publisher('/map_topic', Range, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint_topic', Point, queue_size=1)

    def tick(self):
        #state machine logic goes here
        if self.state == starting:
            self.placements = self.eval.get_placements()
            self.publish_map()
            self.state = flying
            return
        if self.state == flying:
            self.publish_waypoint()
            return
        
    def waypoint_reached(self):
        self.current_sensor += 1
        self.state = dropping

    def dropped(self):
        if self.current_sensor == len(self.placements):
            self.state = returning
            return
        self.state = flying

    def publish_waypoint(self):
        waypoint = Point()
        waypoint.x = self.placements[self.current_sensor][0]
        waypoint.y = self.placements[self.current_sensor][1]
        waypoint.z = 0
        self.waypoint_pub.publish(waypoint)

    def publish_map(self):
        self.map_pub.publish(self.eval.get_map())