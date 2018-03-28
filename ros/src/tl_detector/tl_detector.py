#!/usr/bin/env python
import rospy

from light_classification.tl_classifier import TLClassifier

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

import cv2
import numpy as np
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.bridge = CvBridge()

        self.use_ground_truth = rospy.get_param("~use_ground_truth", default=False)
        self.tl_consideration_distance = rospy.get_param("/tl_consideration_distance",  100)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        if not self.use_ground_truth:
             self.light_classifier = TLClassifier()

        base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.tss = ApproximateTimeSynchronizer([Subscriber("/vehicle/traffic_lights", TrafficLightArray),
                                               Subscriber("/image_color", Image),
                                               Subscriber("/current_pose", PoseStamped)], 30, 0.1)
        self.tss.registerCallback(self.light_image_pose_cb)

        rospy.spin()

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        """
        # test get_waypoint_distance method; TODO: move into rostest
        idx1 = 0
        idx2 = 5
        rospy.logerr("distance between %d and %d: %f", idx1, idx2, self.get_waypoint_distance(idx1, idx2))

        idx1 = 5
        idx2 = 0
        rospy.logerr("distance between %d and %d: %f", idx1, idx2, self.get_waypoint_distance(idx1, idx2))

        idx1 = 50
        idx2 = 60
        rospy.logerr("distance between %d and %d: %f", idx1, idx2, self.get_waypoint_distance(idx1, idx2))

        idx1 = 60
        idx2 = 50
        rospy.logerr("distance between %d and %d: %f", idx1, idx2, self.get_waypoint_distance(idx1, idx2))

        idx1 = len(self.waypoints.waypoints) - 50
        idx2 = 50
        rospy.logerr("distance between %d and %d: %f", idx1, idx2, self.get_waypoint_distance(idx1, idx2))

        idx1 = 50
        idx2 = len(self.waypoints.waypoints) - 50
        rospy.logerr("distance between %d and %d: %f", idx1, idx2, self.get_waypoint_distance(idx1, idx2))
        """

    def light_image_pose_cb(self, traffic_lights_msg, image_msg, pose_msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            traffic_lights_msg (TrafficLightArray): image from car-mounted camera
            image_msg (Image):                      image from car-mounted camera

        """
        self.pose = pose_msg
        self.lights = traffic_lights_msg.lights
        self.camera_image = image_msg

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
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    # TODO: move into common package (might be cool to create a map-class: map = Map(waypoints), map.get_waypoint_distance(..))
    # TODO: in simulation, this takes a non-trivial amount of resources and slowes down the whole pipeline
    def get_closest_waypoint_idx(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = 0
        closest_dist = float("inf")
        for idx, wp in enumerate(self.waypoints.waypoints):
            dx = wp.pose.pose.position.x - pose.position.x
            dy = wp.pose.pose.position.y - pose.position.y
            dist = dx*dx + dy*dy
            if dist < closest_dist:
                closest_idx = idx
                closest_dist = dist

        return closest_idx

    # TODO: move into common package (might cool to create a map-class: map = Map(waypoints), map.get_waypoint_distance(..))
    def get_waypoint_distance(self, wp_idx1, wp_idx2):
        """
        Calculates the the shortest distance between wp1 and wp2. Distance is measured as accumulated distance between the waypoints 
        traversing from wp1 to wp2. This assumes the waypoints form a closed loop. If the distance is negative, wp_idx2 is behind wp_idx1.

         Args:
            wp_idx1 (int): waypoint index of start point
            wp_idx2 (int): waypoint index of end point

        Returns:
            float: signed sortest distance between wp1 and wp2 on the waypoints loop
        """
        assert wp_idx1 >= 0 and wp_idx2 >= 0 and wp_idx1 < len(self.waypoints.waypoints) and wp_idx2 < len(self.waypoints.waypoints)

        distance = 0
        # the signed index-distance
        idx_dist = wp_idx2 - wp_idx1

        # handle the cases where we measure over the beginning of the waypoints loop
        # such that we return always the shortes distance on the loop
        if idx_dist < -len(self.waypoints.waypoints) / 2:
            idx_dist += len(self.waypoints.waypoints)
        elif idx_dist > len(self.waypoints.waypoints) / 2:
            idx_dist += -len(self.waypoints.waypoints)
        
        # the direction (forward vs. backwards)
        dir = np.sign(idx_dist)
        if dir < 0:
            wp_idx1, wp_idx2 = wp_idx2, wp_idx1
        
        idx = wp_idx1
        while idx != wp_idx2:
            p1 = self.waypoints.waypoints[idx].pose.pose.position
            p2 = self.waypoints.waypoints[(idx+1) % len(self.waypoints.waypoints)].pose.pose.position
            dist = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            distance += dist
            
            idx += 1
            if idx >= len(self.waypoints.waypoints):
                idx = 0

        return distance * dir
    
    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.use_ground_truth:
            return light['light'].state

        # Get classification
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        return self.light_classifier.get_classification(light, cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.waypoints:
            rospy.loginfo_throttle(5, "waiting for waypoints callback")
            return -1, TrafficLight.UNKNOWN

        # find the wp idx of the vehicle
        car_closest_wp_idx = self.get_closest_waypoint_idx(self.pose.pose)
        rospy.logdebug("Closest wp to the car's position is %d", car_closest_wp_idx)

        # find the wp idx of the lights and their distance to the car
        relevant_lights = []
        for idx, l in enumerate(self.lights):
            # find the corresponding stop-line (this assumes that the traffic lights and stop lines have the same index in the arrays)
            tl_stop_line_pose = Pose()
            tl_stop_line_pose.position.x = self.config['stop_line_positions'][idx][0]
            tl_stop_line_pose.position.y = self.config['stop_line_positions'][idx][1]
            
            # find closest wp to stop-line
            tl_stop_line_wp_idx = self.get_closest_waypoint_idx(tl_stop_line_pose)
            distance_to_car     = self.get_waypoint_distance(car_closest_wp_idx, tl_stop_line_wp_idx)

            # only consider TLs in front of the car's position, and ones not too far ahead
            if distance_to_car < 0 or distance_to_car > self.tl_consideration_distance:
                rospy.logdebug("Ignoring TL, its behind the car or too far ahead (%.2fm)", distance_to_car)
                continue

            tl = {
                'light': l,
                'stop_line_wp_idx': tl_stop_line_wp_idx,
                'distance': distance_to_car,
                'state': None
            } 
            relevant_lights.append(tl)

        # check if we have any relevant TLs ahead
        if not len(relevant_lights):
            rospy.logdebug("No relevant TL found")
            return -1, TrafficLight.UNKNOWN

        # sort by distance and select the closes one
        relevant_lights.sort(key=lambda x: x['distance'])
        next_relevant_tl = relevant_lights[0]
        rospy.logdebug("Next relevant TL is %.2fm ahead at wp %d.", next_relevant_tl['distance'], next_relevant_tl['stop_line_wp_idx'])

        # find its state
        next_relevant_tl['state'] = self.get_light_state(next_relevant_tl)

        return next_relevant_tl['stop_line_wp_idx'], next_relevant_tl['state']


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
