import rospy
from styx_msgs.msg import TrafficLight
import cv2
import rospkg
import os
import csv


class TLClassifier(object):

    def __init__(self):
        rospack = rospkg.RosPack()
        self.path = os.path.join(rospack.get_path('tl_detector'), "sim_data")
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        f = open(os.path.join(self.path, 'labels.csv'), 'wb')
        self.writer = csv.writer(f)
        self.num_files = 0
        self.last_light = None

    def get_classification(self, light, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        cv2.imshow("input_image", image)
        cv2.waitKey(1)

        if not self.last_light or abs(self.last_light['distance'] - light['distance']) > 1:
            filename = "tl_%05d.png" % self.num_files
            cv2.imwrite(os.path.join(self.path, filename), image)
            rospy.logdebug("Writing image to %s", filename)
            self.writer.writerow([filename, light['light'].state])
            self.last_light = light
            self.num_files += 1


        return light['light'].state
