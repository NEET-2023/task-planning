from generator import *
from evaluator import *
import skimage.measure
import cv2
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

starting = 0
flying = 1
dropping = 2
picking = 3
recharge = 4
returning = 5
done = 6

class TaskPlanner:
    
    def __init__(self, eval: Evaluator):
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
        self.variables, self.height, self.width = eval.get_dims()
        # ROS publishers to execute other nodes
        self.map_pub = rospy.Publisher('/map_topic', OccupancyGrid, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/waypoint_topic', Point, queue_size=1)
        self.place_sensor_pub = rospy.Publisher('/place_sensor', Bool, queue_size=1)
        self.pickup_sensor_pub = rospy.Publisher('/pickup_sensor', Bool, queue_size=1)
        # ROS subscribers to run this script
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.tick)
        self.done_travelling_sub = rospy.Subscriber('done_travelling', Bool, self.waypoint_reached)
        self.sensor_placed_sub = rospy.Subscriber('/sensor_placed', Bool, self.done_dilly_dallying)
        self.sensor_pickedup_sub = rospy.Subscriber('/sensor_pickedup', Bool, self.done_dilly_dallying)

    def tick(self, odom):
        #state machine logic goes here
        while True:
            self.rate.sleep()
            if self.state == starting:
                self.placements = self.eval.get_placements()
                self.publish_occupancy()
                self.set_state(flying)
                continue
            if self.state == flying:
                self.publish_next()
                continue
            if self.state == dropping:
                self.drop()
                continue
            if self.state == picking:
                self.pick()
                continue
            if self.state == recharge:
                continue
            if self.state == returning:
                self.publish_waypoint(self.home)
                continue
        
        
    def waypoint_reached(self, is_reached):
        if is_reached.data:
            if self.state == flying:
                self.current_sensor += 1
            if self.state == returning and self.current_sensor == len(self.placements):
                if self.has_probe:
                    self.final_drop = True
                    self.set_state(dropping)
                    return
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
            if self.final_drop:
                self.set_state(done)
                return
            if self.prev_state == flying:
                self.set_state(returning)
                return
            self.set_state(flying)

    def pick(self):
        pick = Bool()
        pick.data = True
        self.pickup_sensor_pub.publish(pick)

    def publish_next(self):
        waypoint = Point()
        waypoint.x = self.placements[self.current_sensor][0]
        waypoint.y = self.placements[self.current_sensor][1]
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
        occupancy = self.eval.get_occupancy()
        for row in occupancy:
            for value in row:
                grid.data.append(value)
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position = northwest
        self.map_pub.publish(grid)
    
    def set_state(self, new_state):
        self.prev_state = self.state
        self.state = new_state

if __name__ == "__main__":
    path = 'occupancy_grids/images/rolling_hills_map_10.png'
    occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    # cv2.imshow("original occupancy", occupancy_image)
    reduced_occupancy = skimage.measure.block_reduce(occupancy_image, (5, 5), np.max)
    # cv2.imshow('reduced occupancy', reduced_occupancy)
    dilated_occupancy = cv2.dilate(reduced_occupancy, np.ones((7, 7), np.uint8))
    vf = ValueFunction(2, len(occupancy_image), len(occupancy_image[0]), zipper_gen([0.4, 0.6]))

    #info0 = get_gen([[1, 2, 3, 4, 5], [4, 5, 6, 7, 8], [7, 8, 9, 10, 11], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
    #info1 = get_gen([[5, 4, 3, 2, 1], [8, 7, 6, 5, 4], [11, 10, 9, 8, 7], [1, 2, 3, 4, 5], [4, 5, 6, 7, 8]])
    #vf.apply_func(info0, 0)
    #vf.apply_func(info1, 1)
    eval = Evaluator(vf, 5, dilated_occupancy)

    try:
        # create the navigator object, pass in important mapping information
        rospy.init_node('task_planner', anonymous=True)
        PLANNER = TaskPlanner(eval)
        PLANNER.tick()
    except rospy.ROSInterruptException:
        pass
