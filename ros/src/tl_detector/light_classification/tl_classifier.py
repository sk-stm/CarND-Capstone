import rospy
from styx_msgs.msg import TrafficLight
import cv2
import rospkg
import os
import csv


class TLClassifier(object):

    def __init__(self):
        self.probe_groundtruth_data = rospy.get_param("probe_groundtruth_data", False)

        if self.probe_groundtruth_data:
            rospack = rospkg.RosPack()
            self.path = os.path.join(rospack.get_path('tl_detector'), "sim_data")
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            labels_file = os.path.join(self.path, 'labels.csv')
            if os.path.isfile(labels_file) :
                raise RuntimeError("File %s already exists!", labels_file)
            f = open(os.path.join(self.path, 'labels.csv'), 'wb')
            self.writer = csv.writer(f)
            self.num_files = 0
            self.last_light_data = None
            self.new_state = 0

    def get_classification(self, light_data, image):
        """Determines the color of the traffic light in the image

        Args:
            light_data (dict):  wrapped data about the TL
            image (cv::Mat):    image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.probe_groundtruth_data:
            #cv2.imshow("input_image", image)
            #cv2.waitKey(1)

            if not self.last_light_data or  abs(self.last_light_data['distance'] - light_data['distance']) > 1. or self.last_light_data['light'].state != light_data['light'].state:
                # delivered ground-truth state from simulator and image are not in sync, so one has wait a little and delay the probe
                if self.new_state >3:
                    self.new_state = 0
                    filename = "tl_%05d.png" % self.num_files
                    cv2.imwrite(os.path.join(self.path, filename), image)
                    rospy.logdebug("Writing image to %s", filename)
                    self.writer.writerow([filename, light_data['light'].state])
                    self.last_light_data = light_data
                    self.num_files += 1
                else: 
                    self.new_state +=1


        return light_data['light'].state
