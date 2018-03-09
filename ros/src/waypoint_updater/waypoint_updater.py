#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import copy
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

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
STOP_AHEAD_WPS = 0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self._get_current_vel)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None
        self.base_waypoints_length = 0
        self.stop_waypoint_idx_for_traffic_light = None
        self.next_n_waypoints = []
        self.next_n_waypoint_glob_idxs = []
        self.seeing_red_light = False
        self.continue_driving_from_traffic_light = False
        self.already_planed_to_stop = False
        self.car_wp_idx = None
        self.current_vel = 0
        self.car_pos = [0, 0, 0]

        rospy.spin()

    def _get_current_vel(self, curr_vel_twisted_stamped):
        self.current_vel = curr_vel_twisted_stamped.twist.linear.x

    def pose_cb(self, msg):
        """
        Sets the current position of the car
        :param msg: current pose of the car in PoseStamped msg format
        """
        self.car_pos[0] = msg.pose.position.x
        self.car_pos[1] = msg.pose.position.y
        self.car_pos[2] = msg.pose.position.z

        # only calc final waypoints if base waypoints have been published
        if self.base_waypoints:
            self._calculate_next_waypoints()
            self._publish_next_waypoints()

    def _calculate_next_waypoints(self):
        self._calculate_next_waypoints_without_traffic_light()
        self._include_traffic_light_behaviour()

    def _calculate_next_waypoints_without_traffic_light(self):
        """"""
        self.car_wp_idx = self._find_wp_in_front_of_car()

        if not self.next_n_waypoints or self.continue_driving_from_traffic_light:
            self.next_n_waypoints, \
            self.next_n_waypoint_glob_idxs = self._freshly_calc_next_waypoints(car_wp_idx=self.car_wp_idx)
            self.continue_driving_from_traffic_light = False
        else:
            self._update_next_waypoints(next_n_waypoints=self.next_n_waypoints,
                                        next_n_waypoint_glob_idxs=self.next_n_waypoint_glob_idxs,
                                        car_wp_idx=self.car_wp_idx)

    def _freshly_calc_next_waypoints(self, car_wp_idx):
        """
        Calculates the initial waypoints to follow, independent of traffic lights.
        :return: tuple (next waypoints, corresponding global indices)
        """
        next_n_waypoints = []
        next_n_waypoint_glob_idxs = []
        # initial fill
        for i in range(LOOKAHEAD_WPS):
            next_idx = car_wp_idx + i
            next_wp = copy.deepcopy(self.base_waypoints.waypoints[next_idx])
            next_n_waypoints.append(next_wp)
            next_n_waypoint_glob_idxs.append(next_idx)
            # start smoothly when standing
            current_wp_vel = self.get_waypoint_velocity(self.base_waypoints.waypoints[car_wp_idx])
            desired_vel_at_end_of_trajectory = self.get_waypoint_velocity(self.base_waypoints.waypoints[car_wp_idx + LOOKAHEAD_WPS])
            next_wp_vel = current_wp_vel + (current_wp_vel - desired_vel_at_end_of_trajectory) / LOOKAHEAD_WPS
            self.set_waypoint_velocity(next_n_waypoints, -1, next_wp_vel)
        return next_n_waypoints, next_n_waypoint_glob_idxs

    def _update_next_waypoints(self, next_n_waypoints, next_n_waypoint_glob_idxs, car_wp_idx):
        """
        Updates the next n waypoints and corresponding global ids given by reference here according to car position.
        This method is less computational expensive then the full array update.
        :param next_n_waypoints: next waypoints to update
        :param next_n_waypoint_glob_idxs: corresponding global ids to the next waypoints
        :param car_wp_idx: waypoint nearest to the car
        """
        # if waypoints are available
        # find current wp in list of next wps starting at the beginning
        i = 0
        while i < LOOKAHEAD_WPS and next_n_waypoint_glob_idxs[i] < car_wp_idx:
            # remove all entries up to that index
            next_n_waypoints.pop(0)
            next_n_waypoint_glob_idxs.pop(0)
            # append as many as removed at the end
            next_wp = copy.deepcopy(self.base_waypoints.waypoints[car_wp_idx + LOOKAHEAD_WPS + i])
            next_n_waypoints.append(next_wp)
            next_n_waypoint_glob_idxs.append(car_wp_idx + LOOKAHEAD_WPS + i)
            i += 1

    def _include_traffic_light_behaviour(self):
        """"""
        # we can calculate a behaviour for traffic lights if no stop waypoint or base waypoint is present. (for startup)
        if not self.stop_waypoint_idx_for_traffic_light or not self.base_waypoints:
            return

        # if the index is -1, this means there are no red traffic lights detected
        if self.stop_waypoint_idx_for_traffic_light == -1:
            if self.seeing_red_light:
                self.seeing_red_light = False
                self.continue_driving_from_traffic_light = True
                self.already_planed_to_stop = False
            return

        self.seeing_red_light = True
        self.car_wp_idx = self._find_wp_in_front_of_car()

        stop_waypoint_idx_for_traffic_light = self.stop_waypoint_idx_for_traffic_light - STOP_AHEAD_WPS
        if self.car_wp_idx < stop_waypoint_idx_for_traffic_light:
            if not self.already_planed_to_stop:
                self.already_planed_to_stop = True
                self._plan_stop_wps(stop_waypoint_idx_for_traffic_light)
            else:
                self._update_next_waypoints(next_n_waypoints=self.next_n_waypoints,
                                            next_n_waypoint_glob_idxs=self.next_n_waypoint_glob_idxs,
                                            car_wp_idx=self.car_wp_idx)

        else:
            # we just drove over the stop line ut the traffic light is still red -> set all next wps to 0 vel
            for i in range(LOOKAHEAD_WPS):
                self.set_waypoint_velocity(self.next_n_waypoints, i, 0)

    def _plan_stop_wps(self, stop_waypoint_idx_for_traffic_light):
        # the stop lane is still in front of us
        number_of_wps_between_car_and_stop_lane = stop_waypoint_idx_for_traffic_light - self.car_wp_idx
        for current_wp_number_in_next_wps_array in range(number_of_wps_between_car_and_stop_lane):
            number_of_wp_until_stop = number_of_wps_between_car_and_stop_lane
            desired_wp_vel = self._calculate_decreasing_velocity(number_of_wp_until_stop,
                                                                 current_wp_number_in_next_wps_array)
            self.set_waypoint_velocity(self.next_n_waypoints,
                                       current_wp_number_in_next_wps_array,
                                       max(desired_wp_vel, 0))

        # from the stop_waypoint we stoped ahead from we should also stop
        for i in range(number_of_wps_between_car_and_stop_lane, STOP_AHEAD_WPS):
            self.set_waypoint_velocity(self.next_n_waypoints,
                                       self.stop_waypoint_idx_for_traffic_light - self.car_wp_idx - i, 0)

    def _calculate_decreasing_velocity(self, number_of_wps_until_stop_lane, wp_number_in_next_wps):
        """"""
        decrement_step = self.current_vel / number_of_wps_until_stop_lane
        return decrement_step * (number_of_wps_until_stop_lane - wp_number_in_next_wps)

    def _publish_next_waypoints(self):
        """"""
        self.final_waypoints_pub.publish(Lane(waypoints=self.next_n_waypoints))

    def waypoints_cb(self, waypoints):
        """
        Gets a giant list of waypoints in the world in Lane msg format.
        :param waypoints: waypoint list
        """
        self.base_waypoints = waypoints
        self.base_waypoints_length = len(self.base_waypoints.waypoints)

    def _find_wp_in_front_of_car(self):
        """
        Finds the waypoint in front of the car in the list of waypoints.
        This method assumes that the car is oriented towards tex direction of the coordinate frame.
        :return: the wp in front of the car
        """
        # find closest wp to the car where wp.x > car_pos_x
        closest_idx = self.car_wp_idx
        i = 1
        closest_dist = float("inf")

        if self.car_wp_idx is not None:
            closest_dist = self._wp_car_dist(self.base_waypoints.waypoints[self.car_wp_idx])
            next_wp_idx = (self.car_wp_idx + i) % self.base_waypoints_length
            next_wp_car_dist = self._wp_car_dist(self.base_waypoints.waypoints[next_wp_idx])
            while closest_dist > next_wp_car_dist:
                closest_dist = next_wp_car_dist
                i += 1
                next_wp_idx = (self.car_wp_idx + i) % self.base_waypoints_length
                next_wp_car_dist = self._wp_car_dist(self.base_waypoints.waypoints[next_wp_idx])
                closest_idx = (self.car_wp_idx + i) % self.base_waypoints_length
        else:
            # initial search in all the waypoints
            for idx, wp in enumerate(self.base_waypoints.waypoints):
                curr_dist = self._wp_car_dist(wp)
                if curr_dist < closest_dist:
                    closest_idx = idx
                    closest_dist = curr_dist

        return closest_idx

    def _wp_car_dist(self, wp):
        """
        Calculates the distance of a waypoint to the current position of the car
        :param wp: waypoint to calc the dist to
        :return: dist between wp and car
        """
        return math.sqrt((wp.pose.pose.position.x - self.car_pos[0]) ** 2 +
                         (wp.pose.pose.position.y - self.car_pos[1]) ** 2 +
                         (wp.pose.pose.position.z - self.car_pos[2]) ** 2)

    def traffic_cb(self, msg):
        """
        Receives the message of the traffic light detector containing waypointindex to stop at.
        If the index is -1, no red right was detected.
        :param msg: traffic light detection message msg.data contains the index in uint8 format
        """
        self.stop_waypoint_idx_for_traffic_light = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint_idx, velocity):
        waypoints[waypoint_idx].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        """
        Calculates the distance from wp1 to wp2
        :param waypoints: waypointlist containing all waypoints
        :param wp1: index of the first waypoint
        :param wp2: intext of the second waypoint
        :return: distance between wp1 and wp2
        """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
