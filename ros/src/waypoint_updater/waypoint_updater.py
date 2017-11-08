#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped
import tf

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MPH_TO_MPS = 0.44704
MAX_SPEED = 20 * MPH_TO_MPS

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
	rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb) # Provide linear velocity

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.car_pose = None
        self.waypoint_list = None
        self.yaw = None
        self.red_traffic_light_waypoint_index = None
        self.current_velocity = None
        self.loop()
        rospy.spin()

    # Callback functions
    def pose_cb(self, msg):
        self.car_pose = msg
        orientation = msg.pose.orientation
        orientation_quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _,_,self.yaw = tf.transformations.euler_from_quaternion(orientation_quaternion)

    def waypoints_cb(self, waypoints):
        self.waypoint_list = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_traffic_light_waypoint_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self,msg):
        self.current_velocity = msg

    # Publish final_waypoints
    def loop(self):
        rate = rospy.Rate(10) # 40Hz
        while not rospy.is_shutdown():
            if (self.car_pose is None) or (self.waypoint_list is None):
                continue

            frame_id = self.waypoint_list.header.frame_id
            lane_start = self.NextWaypoint()
            waypoint_list = self.waypoint_list.waypoints[lane_start:lane_start + LOOKAHEAD_WPS]

            if(self.current_velocity is not None):
                current_linear_vel = self.current_velocity.twist.linear.x
                print("lane_start", lane_start, "red_light",self.red_traffic_light_waypoint_index, 
                        "current_linear_vel",current_linear_vel)

            for waypoint in waypoint_list:
                self.set_waypoint_velocity(waypoint, MAX_SPEED)

            if lane_start + LOOKAHEAD_WPS >= len(self.waypoint_list.waypoints):
                for waypoint in waypoint_list[-10:]:
                    self.set_waypoint_velocity(waypoint, 0)

            if self.red_traffic_light_waypoint_index and self.red_traffic_light_waypoint_index > 0:
                red_traffic_light_distance = max(0, self.red_traffic_light_waypoint_index - lane_start)
                #min is needed else we get IndexError: list index out of range
                red_traffic_light_distance = min(red_traffic_light_distance, len(waypoint_list)-1)
                full_stop = waypoint_list[red_traffic_light_distance]
                distance_to_traffic_light = max(0, self.cal_distance(waypoint_list[0], full_stop.pose)-2)
                print("distance to stop",distance_to_traffic_light)

                for waypoint_index, waypoint in enumerate(waypoint_list):
                    distance_to_traffic_light = self.cal_distance(waypoint, full_stop.pose)
                    distance_to_traffic_light = max(0, distance_to_traffic_light - 2)
                    velocity = math.sqrt(distance_to_traffic_light)
                    if velocity < 1.0:
                        waypoint.twist.twist.linear.x = 0
                    elif velocity < waypoint.twist.twist.linear.x:
                        waypoint.twist.twist.linear.x = velocity

            lane = self.get_lane_msg(frame_id, waypoint_list)
            self.final_waypoints_pub.publish(lane)
            rate.sleep()

    # Helper functions
    def get_waypoint_velocity(self, waypoint):
        return waypoint.waypoints.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def get_lane_msg(self, frame_id, waypoints):
        lane = Lane()
        lane.header.frame_id = frame_id
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        return lane

    # Compute the distance between waypoint the current car position
    def cal_distance(self, waypoint, car_pose):
        wp_x = waypoint.pose.pose.position.x
        wp_y = waypoint.pose.pose.position.y
        wp_z = waypoint.pose.pose.position.z
        car_x = car_pose.pose.position.x
        car_y = car_pose.pose.position.y
        car_z = car_pose.pose.position.z
        return math.sqrt((wp_x-car_x)**2 + (wp_y-car_y)**2 + (wp_z-car_z)**2)

    # We have a list of all these waypoints around our highway, we can see which is closest to us.
    def ClosestWaypoint(self):
        closest_distance = 100000.0 # large number
        closestWaypoint_index = 0
        start = 0
        end = len(self.waypoint_list.waypoints)
        for i in range(start, end):
            dist = self.cal_distance(self.waypoint_list.waypoints[i],self.car_pose)
            if(dist < closest_distance):
                closestWaypoint_index = i
                closest_distance = dist
        return closestWaypoint_index

    # The difference between closest_waypoint and the next_waypoint; sometimes the closest waypoint maybe at your back.
    def NextWaypoint(self):
        closest_waypoint_index = self.ClosestWaypoint()
        #number_waypoints = len(self.waypoint_list)
        waypoint_x = self.waypoint_list.waypoints[closest_waypoint_index].pose.pose.position.x
        waypoint_y = self.waypoint_list.waypoints[closest_waypoint_index].pose.pose.position.x
        delta_x = waypoint_x - self.car_pose.pose.position.x
        delta_y = waypoint_y - self.car_pose.pose.position.y
        heading = math.atan2(delta_y, delta_x)
        angle = abs(self.yaw - heading)
        if angle > math.pi / 4:
            closest_waypoint_index = closest_waypoint_index + 1
        return closest_waypoint_index

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
