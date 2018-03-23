import rospy
from styx_msgs.msg import TrafficLight
import cv2
import rospkg
import os
import csv
import keras
import numpy as np
import tensorflow as tf


class TLClassifier(object):

    def __init__(self):
        self.last_light = None

        self.gen_train_data = rospy.get_param("~gen_train_data", default=False)
        
        if self.gen_train_data:
            self.num_files = 0
            rospack = rospkg.RosPack()
            self.path = os.path.join(rospack.get_path('tl_detector'), "sim_data")
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            f = open(os.path.join(self.path, 'labels.csv'), 'wb')
            self.writer = csv.writer(f)
        else:
            # TODO need to add check that the model is compatible with current keras version
            self.model = keras.models.load_model("model.h5")
            self.model._make_predict_function()
            rospy.loginfo("keras model loaded")
            self.graph = tf.get_default_graph()

    def get_classification(self, light, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # read and save the light state immediately because it can change
        # by the time we save the image
        lstate = light['light'].state

        # Reduce image to to 1/4 size
        rs_image = cv2.resize(image, None, fx=0.5, fy=0.5)
        # rs_image = image
        

        if self.gen_train_data:
            # cv2.imshow("input_image", image)
            # cv2.waitKey(1)

            if not self.last_light or abs(self.last_light['distance'] - light['distance']) > 1:
                filename = "tl_%05d.png" % self.num_files
                cv2.imwrite(os.path.join(self.path, filename), rs_image)
                rospy.logdebug("Writing image to %s", filename)
                self.writer.writerow([filename, lstate])
                self.last_light = light
                self.num_files += 1

            return lstate

        else:
            image_array = np.asarray(rs_image)
            # rospy.loginfo("Calling model prediction image shape %s", np.shape(image_array))
            with self.graph.as_default():
                light_predict = self.model.predict(image_array[None, :, :, :], batch_size=1)
            lightval = np.argmax(light_predict)
            rospy.logdebug("Light prediction %s, pred: %d, state: %d", light_predict, lightval, light['light'].state)
            if lightval != light['light'].state:
                rospy.loginfo("Light prediction %s, pred: %d, state: %d", light_predict, lightval, light['light'].state)
            return lightval
