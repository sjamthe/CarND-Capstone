#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
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

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32 ,self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.car_pose = None
        self.waypoint_list = None
        self.yaw = None
        self.trafficstop_wp = None
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
        self.trafficstop_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Publish final_waypoints
    def loop(self):
        rate = rospy.Rate(40) # 40Hz
	#rospy.loginfo("Starting loop")
        while not rospy.is_shutdown():
            if (self.car_pose is None) or (self.waypoint_list is None):
                continue

            frame_id = self.waypoint_list.header.frame_id
            lane_start = self.NextWaypoint()
            lane_end = lane_start + LOOKAHEAD_WPS
            waypoint_list = self.waypoint_list.waypoints[lane_start:lane_end]

            if(self.trafficstop_wp > 0):
                print("lane_Start :", lane_start, "self.trafficstop_wp",self.trafficstop_wp)
            
            #do we need to stop/slow down?
            if(self.trafficstop_wp >= lane_start and self.trafficstop_wp <= lane_end):
                slow_len = self.trafficstop_wp - lane_start - 15
                old_speed = MAX_SPEED
                if(lane_start > 0):
                    old_speed = self.get_waypoint_velocity(self.waypoint_list.waypoints[lane_start-1])
                print("slow_len",slow_len,"old_speed",old_speed)

            current = 0
            for waypoint in waypoint_list:
                if(self.trafficstop_wp >= lane_start and self.trafficstop_wp <= lane_end):
                    if(current >= slow_len):
                        self.set_waypoint_velocity(waypoint, 0)
                    else:
                        speed = old_speed*(1.0-float(current)/float(slow_len))
                        if(speed < 0):
                            speed = 0
                        self.set_waypoint_velocity(waypoint, speed)
                        print("setting speed ", speed, "for cur",current)
                else:                    
                    self.set_waypoint_velocity(waypoint, MAX_SPEED)
                current = current + 1

            if lane_start + LOOKAHEAD_WPS >= len(self.waypoint_list.waypoints):
                for waypoint in waypoint_list[-10:]:
                    self.set_waypoint_velocity(waypoint, 0)
            lane = self.get_lane_msg(frame_id, waypoint_list)

            self.final_waypoints_pub.publish(lane)	
            rate.sleep()

    # Helper functions
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity
        #print("setting velocity", velocity)

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
