from generator import *
from evaluator import *
import cv2
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import OccupancyGrid

starting = 0
flying = 1
dropping = 2
picking = 3
returning = 5

class TaskPlanner:
    
    def __init__(self, eval: Evaluator):
        self.resolution = 1
        self.northedge = 0
        self.westedge = 0

        self.state = starting
        self.placements = []
        self.current_sensor = 0
        self.eval = eval
        self.variables, self.height, self.width = eval.get_dims()
        self.map_pub = rospy.Publisher('/map_topic', Range, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint_topic', Point, queue_size=1)

    def tick(self):
        #state machine logic goes here
        if self.state == starting:
            self.placements = self.eval.get_placements()
            self.publish_occupancy()
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

    def publish_occupancy(self):
        northwest = Point()
        northwest.x = self.northedge
        northwest.y = self.westedge
        northwest.z = 0
        grid = OccupancyGrid()
        occupancy = self.eval.get_occupancy()
        for row in occupancy:
            for value in row:
                grid.data.append(value)
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position = northwest
        self.map_pub.publish(grid)

if __name__ == "__main__":
    path = '../occupancy_grids/images/rolling_hills_map_10.png'
    occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    vf = ValueFunction(2, len(occupancy_image), len(occupancy_image[0]), zipper_gen([0.4, 0.6]))

    info0 = get_gen([[1, 2, 3, 4, 5], [4, 5, 6, 7, 8], [7, 8, 9, 10, 11], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
    info1 = get_gen([[5, 4, 3, 2, 1], [8, 7, 6, 5, 4], [11, 10, 9, 8, 7], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
    vf.apply_func(info0, 0)
    vf.apply_func(info1, 1)
    eval = Evaluator(vf, 5, occupancy_image)

    try:
        # create the navigator object, pass in important mapping information
        rospy.init_node('task_planner', anonymous=True)
        PLANNER = TaskPlanner(eval)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass