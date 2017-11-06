#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.traffic_light_waypoint_indexes = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        stop_line_positions = self.config['stop_line_positions']

        # self.traffic_lights = []

        for line in stop_line_positions:
            # print("line:" , line)
            traffic_light = TrafficLight()
            traffic_light.pose = PoseStamped()
            traffic_light.pose.pose.position.x = line[0]
            traffic_light.pose.pose.position.y = line[1]
            traffic_light.pose.pose.position.z = 0.0
            # traffic_lights.append(traffic_light)
            self.traffic_light_waypoint_indexes.append(self.get_closest_waypoint(traffic_light.pose.pose))
            # print("self.traffic_light_waypoint_indexes:", self.traffic_light_waypoint_indexes)
    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD: # unknow time
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def cal_distance(self, waypoint, car_pose):
        wp_x = waypoint.pose.pose.position.x
        wp_y = waypoint.pose.pose.position.y
        wp_z = waypoint.pose.pose.position.z
        car_x = car_pose.position.x
        car_y = car_pose.position.y
        car_z = car_pose.position.z
        return math.sqrt((wp_x-car_x)**2 + (wp_y-car_y)**2 + (wp_z-car_z)**2)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_distance = 100000.0  # large number
        closestWaypoint_index = 0
        # print("len(self.waypoints.waypoints):", len(self.waypoints.waypoints))
        waypoints_waypoints_size = len(self.waypoints.waypoints)
        for i in range(waypoints_waypoints_size):
            dist = self.cal_distance(self.waypoints.waypoints[i], pose)
            if(dist < closest_distance):
                closestWaypoint_index = i
                closest_distance = dist
        return closestWaypoint_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # No traffic light
        if(not self.has_image):
            self.prev_light_loc = None

            return False
        # A traffic light
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8") # ROS image change to OpenCV image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        singal_image = np.zeros((500, 500,3), np.uint8)

        # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints):
            car_position_index = self.get_closest_waypoint(self.pose.pose)



        #TODO find the closest visible traffic light (if one exists)


        for traffic_light_waypoint_indexe in self.traffic_light_waypoint_indexes:
            if traffic_light_waypoint_indexe > car_position_index:
                light = True
                light_wp = traffic_light_waypoint_indexe
                break

        if light:
            state = self.get_light_state(light)
            #print("Traffic Light State", state)
            if state == 0:
                print ("RED")
            else:
                print("GREEN")
            return light_wp, state
        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':

    try:
        TLDetector()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
