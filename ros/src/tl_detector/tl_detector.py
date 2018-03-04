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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.stop_line_wp_idx = -1
        self._old_dist_betw_tl_and_car = float("inf")
        self._curr_dist_car_tl = float("inf")
        self._closest_tl_idx = 0

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
        self.tl_wp_idx_ahead = None
        self.last_tl_idx_ahead = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

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
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest(self, wpts, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            position (Position): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # find closest wp to the car where wp.x > car_pos_x
        closest_idx = 0
        closest_dist = float("inf")
        for idx, wp in enumerate(wpts):
            curr_dist = self._calc_dist(wp.pose.pose.position, position)
            if curr_dist < closest_dist:
                closest_idx = idx
                closest_dist = curr_dist

        return closest_idx, closest_dist

    def _calc_wp_dist(self, idx1, idx2):
        """
        Calculates the distance between waypoints in the waypoints list attribute.
        :param idx1: index of the first waypoint in the list.
        :param idx2: index of the second waypoint in the list.
        :return: distance between the two waypoints
        """
        return self._calc_dist(self.waypoints.waypoints[idx1].pose.pose.position,
                               self.waypoints.waypoints[idx2].pose.pose.position)

    def _calc_tl_wp_dist(self, tl_idx, wp_idx):
        """
        Calculates the distance between a traffic ligt position and a waypoint
        :param tl_idx: index of the traffic light in the traffic light list.
        :param wp_idx: index of the waypoint in the waypoint list
        :return: distance between the two points
        """
        return self._calc_dist(self.lights[tl_idx].pose.pose.position,
                               self.waypoints.waypoints[wp_idx].pose.pose.position)

    def _calc_dist(self, position, position2):
        """
        Calculates the distance of a waypoint to the current position of the car
        :param position: position do calc the dist between
        :param position": second position to calc the dist between
        :return: dist between position and position2
        """
        return math.sqrt((position.x - position2.x) ** 2 +
                         (position.y - position2.y) ** 2 +
                         (position.z - position2.z) ** 2)

    def _calc_dist_2d(self, px, py, wp_idx):
        """
        Calculates the disttance in 2D (X,Y) of a point in the world(px, py) and a waypoint in the waypoint list.
        :param px: x coordinate of the point
        :param py: y coordinate of the point
        :param wp_idx: index of the waypoint in the waypoint list
        :return: distance between the point and waypoint
        """
        return math.sqrt(
            (px - self.waypoints.waypoints[wp_idx].pose.pose.position.x) ** 2 +
            (py - self.waypoints.waypoints[wp_idx].pose.pose.position.y) ** 2)


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # TODO Get classification
        #return self.light_classifier.get_classification(cv_image)
        return light.state

    def _find_waypoint_next_to_tl(self, search_idx, tl_idx):
        """
        Finds the nearest waypoint to a traffic light
        :param search_idx: waypoint index to start searching at
        :param tl_idx: index of the closest traffic light (in the traffic light array)
        :return: closest waypoint to the traffic light, index of the nearest waypoint
        """
        best_wp_tl_dist = float("inf")
        curr_wp_tl_dist = self._calc_tl_wp_dist(tl_idx=tl_idx, wp_idx=search_idx)
        i = 1
        while curr_wp_tl_dist < best_wp_tl_dist:
            best_wp_tl_dist = curr_wp_tl_dist
            curr_wp_tl_dist = self._calc_tl_wp_dist(tl_idx=tl_idx, wp_idx=search_idx + i)
            i += 1

        return search_idx + i

    def find_wp_for_stopline_to_next_tl(self, stop_line_pos, light_wp_idx):
        """
        Finds the waypoint nearest to the stop line that belongs to the traffic light near the with the given
         waypoint index.
        :param stop_line_pos: position of the stop line
        :param light_wp_idx: waypoint of the traffic light
        :return: index of the waypoint next to the stop line
        """
        i = 0
        min_dist = float("inf")
        stop_line_wp_dist = 1
        wp_idx = 0
        while stop_line_wp_dist <= min_dist:
            stop_line_wp_dist = self._calc_dist_2d(stop_line_pos[0], stop_line_pos[1], light_wp_idx + i)
            if stop_line_wp_dist < min_dist:
                min_dist = stop_line_wp_dist
                wp_idx = light_wp_idx + i
                i -= 1

        return wp_idx

    def _get_next_tl_along_waypoints(self, car_wp_idx, stop_line_positions):
        """
        Search for next traffic light along the waypoints starting at the position of the car.
        This method assumes that all traffic lights in the traffic light list are relevant for the car.
        But it assumes nothing about the order or location of the traffic lights and might be overhead if the traffic
        lights are ordered along the waypoints.
        :param car_wp_idx:
        :return:
        """
        i = 1
        closest_tl_idx = self._closest_tl_idx
        current_closest_tl_idx = closest_tl_idx
        while closest_tl_idx == current_closest_tl_idx:
            # search along waypoints until another traffic light is found
            closest_tl_idx, closest_tl_dist = self.get_closest(self.lights,
                                                               self.waypoints.waypoints[car_wp_idx + i].pose.pose.position)
            i += 1
        light_wp_idx = self._find_waypoint_next_to_tl(car_wp_idx + i, closest_tl_idx)
        # this assumes that the traffic lights and stop lines have the same index in the array
        stop_line_wp_idx = self.find_wp_for_stopline_to_next_tl(stop_line_positions[closest_tl_idx], light_wp_idx)

        return light_wp_idx, closest_tl_idx, stop_line_wp_idx

    def _get_closest_tl_ahead_of_car(self, car_wp_idx, stop_line_positions):
        """
        Gets the closest traffic light in front of the car. This method assumes, that all traffic lights
        in the traffic lights array are facing the car.
        :param car_wp_idx: waypoint index next to the car
        :return: traffic light ahead of the car (might not be seen yet)
        """
        closest_tl_idx, closest_tl_dist = self.get_closest(self.lights, self.waypoints.waypoints[car_wp_idx].pose.pose.position)

        # test if closest traffic light is in front of car
        nearer_wp_dist = self._calc_tl_wp_dist(tl_idx=closest_tl_idx, wp_idx=car_wp_idx+1)
        if nearer_wp_dist < closest_tl_dist:
            # the traffic light is in front of the car
            # find waypoint near the closest traffic light ahead of the car
            light_wp_idx = self._find_waypoint_next_to_tl(car_wp_idx, closest_tl_idx)
            # this assumes that the traffic lights and stop lines have the same index in the array
            stop_line_wp_idx = self.find_wp_for_stopline_to_next_tl(stop_line_positions[closest_tl_idx], light_wp_idx)
        else:
            # nearest traffic light is behind the car
            light_wp_idx, \
            closest_tl_idx, \
            stop_line_wp_idx = self._get_next_tl_along_waypoints(car_wp_idx, stop_line_positions)

        return light_wp_idx, closest_tl_idx, stop_line_wp_idx

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose is not None and self.waypoints is not None:
            car_wp_idx, _ = self.get_closest(self.waypoints.waypoints, self.pose.pose.position)

            # TODO find the closest visible traffic light (if one exists)

            # TODO This code seems rather complicated. Is that necessary?
            if self.lights:
                if not self.tl_wp_idx_ahead:
                    # initial search for next traffic light
                    next_tl_wp_idx, \
                    next_tl_idx, \
                    stop_line_wp_idx = self._get_closest_tl_ahead_of_car(car_wp_idx, stop_line_positions)
                    self.tl_wp_idx_ahead = next_tl_wp_idx
                    self._closest_tl_idx = next_tl_idx
                    self.stop_line_wp_idx = stop_line_wp_idx

                # regularly update distance between car an traffic light
                self._curr_dist_car_tl = self._calc_wp_dist(self.tl_wp_idx_ahead, car_wp_idx)

                if self._curr_dist_car_tl > self._old_dist_betw_tl_and_car:
                    # search for next traffic if the car passed the traffic light
                    next_tl_wp_idx, \
                    next_tl_idx, \
                    stop_line_wp_idx = self._get_next_tl_along_waypoints(car_wp_idx, stop_line_positions)
                    self.tl_wp_idx_ahead = next_tl_wp_idx
                    self._closest_tl_idx = next_tl_idx
                    self.stop_line_wp_idx = stop_line_wp_idx
                    # calc new distance because waypoint to traffic light changed
                    self._curr_dist_car_tl = self._calc_wp_dist(self.tl_wp_idx_ahead, car_wp_idx)

                # TODO set this number somewhere else
                if self._curr_dist_car_tl <= 50 + 25:
                    # if traffic light is 100 meter away
                    light = self.lights[self._closest_tl_idx]

                self._old_dist_betw_tl_and_car = self._curr_dist_car_tl

                if light:
                    state = self.get_light_state(light)
                    return self.stop_line_wp_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
