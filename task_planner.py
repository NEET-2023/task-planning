from generator import *
from evaluator import *
import math
import skimage.measure
import cv2
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int16

starting = 0
flying = 1
dropping = 2
picking = 3
recharge = 4
returning = 5
mission_complete = 6

world_dims = (-100, 100, -100, 100)
max_row = 0
max_col = 0

class TaskPlanner:
    
    def __init__(self, eval: Evaluator, motion_occupancy):
        self.home = Point()
        self.home.x = 0
        self.home.y = 0
        self.home.z = 0
        self.has_probe = True
        self.final_drop = False
        self.resolution = 1
        self.northedge = 0
        self.westedge = 0
        self.rate = rospy.Rate(1)
        self.prev_state = None
        self.state = starting
        self.placements = []
        self.current_sensor = 0
        self.eval = eval
        self.motion_occupancy = motion_occupancy
        self.variables, self.height, self.width = eval.get_dims()
        # ROS publishers to execute other nodes
        self.map_pub = rospy.Publisher('/map_topic', OccupancyGrid, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint_topic', Point, queue_size=1)
        self.place_sensor_pub = rospy.Publisher('/place_sensor', Bool, queue_size=1)
        self.pickup_sensor_pub = rospy.Publisher('/pickup_sensor', Bool, queue_size=1)
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1)
        self.prev_state_pub = rospy.Publisher('/prev_state', Int16, queue_size=1)
        self.pod_index_pub = rospy.Publisher('/pod_index', Int16, queue_size=1)
        # ROS subscribers to run this script
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.tick)
        self.done_travelling_sub = rospy.Subscriber('done_travelling', Bool, self.waypoint_reached)
        self.sensor_placed_sub = rospy.Subscriber('/sensor_placed', Bool, self.done_dilly_dallying)
        self.sensor_pickedup_sub = rospy.Subscriber('/sensor_pickedup', Bool, self.done_dilly_dallying)

    def publish_state(self):
        wrapper = Int16()
        wrapper.data = self.state
        self.state_pub.publish(wrapper)

    def publish_prev_state(self):
        wrapper = Int16()
        wrapper.data = self.state
        self.prev_state_pub.publish(wrapper)

    def publish_pod_index(self):
        wrapper = Int16()
        wrapper.data = self.current_sensor
        self.pod_index_pub.publish(wrapper)

    def tick(self, odom):
        #state machine logic goes here
        self.publish_state()
        self.publish_prev_state()
        self.publish_pod_index()
        self.publish_occupancy()
        #self.publish_occupancy()
        if self.state == starting:
            print("Generating Placements")
            self.placements = self.eval.get_placements()
            self.set_state(flying)
            return
        if self.state == flying:
            self.publish_next()
            return
        if self.state == dropping:
            self.drop()
            return
        if self.state == picking:
            self.pick()
            return
        if self.state == recharge:
            return
        if self.state == returning:
            self.publish_waypoint(self.home)
            return
        
        
    def waypoint_reached(self, is_reached):
        if is_reached.data:
            if self.state == dropping or self.state == picking:
                return
            if self.state == returning and self.current_sensor == len(self.placements - 1): #gone through sensor list
                if self.has_probe: #returned to base after picking up the last sensor
                    self.final_drop = True
                    self.set_state(dropping)
                else: #returned to base after placing the last sensor
                    self.current_sensor = 0
                    self.set_state(flying)
            elif self.has_probe:
                self.set_state(dropping)
            else:
                self.set_state(picking)

    def drop(self):
        drop = Bool()
        drop.data = True
        self.place_sensor_pub.publish(drop)

    def done_dilly_dallying(self, done):
        if done.data:
            if self.state == dropping:
                self.current_sensor += 1
            if self.state == dropping or self.state == picking:
                self.has_probe = not self.has_probe
            if self.final_drop:
                self.set_state(mission_complete)
                return
            if self.prev_state == flying:
                self.set_state(returning)
            elif self.prev_state == returning:
                self.set_state(flying)

    def pick(self):
        pick = Bool()
        pick.data = True
        self.pickup_sensor_pub.publish(pick)

    def publish_next(self):
        waypoint = Point()
        (x, y) = (self.placements[self.current_sensor][0], self.placements[self.current_sensor][1])
        (new_x, new_y) = grid_to_meters(world_dims, max_row, max_col, x, y)
        waypoint.x = new_x
        waypoint.y = new_y
        waypoint.z = 0
        self.publish_waypoint(waypoint)

    def publish_waypoint(self, waypoint):
        self.waypoint_pub.publish(waypoint)

    def publish_occupancy(self):
        northwest = Point()
        northwest.x = self.northedge
        northwest.y = self.westedge
        northwest.z = 0
        grid = OccupancyGrid()
        occupancy = self.motion_occupancy
        for row in occupancy:
            for value in row:
                grid.data.append(int(value/3))
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position = northwest
        self.map_pub.publish(grid)
    
    def set_state(self, new_state):
        self.prev_state = self.state
        self.state = new_state

# Functions for testing purposes
def grid_to_meters(world_dims: tuple, max_row: int, max_col: int, row: int, col: int) -> tuple:
    """
    Takes in a grid coordinate and returns a location in meters.

    Parameters:
    row, col (int): a row, col location in the discretized state space

    Returns:
    x, y (float): a x, y location in the continuous state space
    """
    min_x, max_x, min_y, max_y = world_dims
    x = (max_x - min_x)*row/max_row + min_x
    y = (max_y - min_y)*col/max_col + min_y
    return x, y

if __name__ == "__main__":
    path = 'occupancy_grids/images/rolling_hills_map_10.png'
    occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    # cv2.imshow("original occupancy", occupancy_image)
    reduced_occupancy = skimage.measure.block_reduce(occupancy_image, (5, 5), np.max)
    # cv2.imshow('reduced occupancy', reduced_occupancy)
    motion_dilated_occupancy = cv2.dilate(reduced_occupancy, np.ones((7, 7), np.uint8))
    dilated_occupancy = cv2.dilate(reduced_occupancy, np.ones((11, 11), np.uint8))
    vf = ValueFunction(2, len(dilated_occupancy), len(dilated_occupancy[0]), zipper_gen([0.4, 0.6]))

    info0 = sin_gen(100, 1, 2, 0.1, 0.1)
    info1 = planar_gen(0, 0, 100, 100)
    poi1 = grad_gen(250, 200, 300)
    poi2 = grad_gen(260, 175, 300)
    vf.apply_func(info0, 0)
    vf.apply_func(info1, 1)
    vf.apply_func(poi1, 0)
    vf.apply_func(poi2, 1)
    eval = Evaluator(vf, 5, dilated_occupancy, 1000)
    max_row, max_col = np.array(dilated_occupancy.shape) - 1

    try:
        # create the navigator object, pass in important mapping information
        rospy.init_node('task_planner', anonymous=True)
        PLANNER = TaskPlanner(eval, motion_dilated_occupancy)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
