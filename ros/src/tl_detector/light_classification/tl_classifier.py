import rospy
import cv2
import rospkg
import os
import csv
import keras
import numpy as np
import tensorflow as tf

DETECTION_TH = 0.5

class TLClassifier(object):

    def __init__(self):
        self.last_light = None

        self.gen_train_data = rospy.get_param("~gen_train_data", default=False)
        self.use_real_world_classifier = rospy.get_param("~real_world_classifier", default=False)
        
        if self.gen_train_data:
            self.num_files = 0
            rospack = rospkg.RosPack()
            self.path = os.path.join(rospack.get_path('tl_detector'), "sim_data")
            if not os.path.exists(self.path):
                os.makedirs(self.path)
            f = open(os.path.join(self.path, 'labels.csv'), 'wb')
            self.writer = csv.writer(f)
        else:
            if self.use_real_world_classifier:
                # real world classifier
                self.real_world_detection_graph = tf.Graph()
                with self.real_world_detection_graph.as_default():
                    od_graph_def = tf.GraphDef()
                    with tf.gfile.GFile('frozen_real_world_detection_graph.pb', 'rb') as fid:
                        od_graph_def.ParseFromString(fid.read())
                        tf.import_graph_def(od_graph_def, name='')
                    self.real_world_detector_input_tensor = self.real_world_detection_graph.get_tensor_by_name('image_tensor:0')
                    self.real_world_detection_scores = self.real_world_detection_graph.get_tensor_by_name('detection_scores:0')
                    self.real_world_detection_classes = self.real_world_detection_graph.get_tensor_by_name('detection_classes:0')
                self.sess = tf.Session(graph=self.real_world_detection_graph)
                rospy.loginfo("real world detection model loaded")
            else:
                # simulation classifier
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
        if not self.use_real_world_classifier:
            rs_image = cv2.resize(image, None, fx=0.5, fy=0.5)
        else:
            rs_image = image

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
            if self.use_real_world_classifier:
                with self.real_world_detection_graph.as_default():
                    # Expand dimension since the model expects image to have shape [1, None, None, 3].
                    expanded_image = np.expand_dims(rs_image, axis=0)
                    scores, classes = self.sess.run(
                        [self.real_world_detection_scores, self.real_world_detection_classes],
                        feed_dict={self.real_world_detector_input_tensor: expanded_image})
                    tl_state = self._interpret_real_world_detection_results(classes=classes, scores=scores)
                    rospy.logdebug("tl_classifier: tl_state = %s", tl_state)
                return tl_state
            else:
                image_array = np.asarray(rs_image)
                with self.graph.as_default():
                    light_predict = self.model.predict(image_array[None, :, :, :], batch_size=1)
                lightval = np.argmax(light_predict)
                rospy.logdebug("Returned from model prediction %s, pred: %d, state: %d", light_predict, lightval, light['light'].state)
                return lightval

    def _interpret_real_world_detection_results(self, classes, scores):
        """
        Maps classification labels: UNKNOWN = 4 GREEN = 2 YELLOW = 1 RED = 0
        to udacity traffic light labels UNKNOWN: = 4 GREEN = 3 YELLOW = 2 RED = 1
        :param classes:
        :param scores:
        :return:
        """
        # currently we only evaluate the most certain one and this also only if it's more certain than 50%
        if scores[0][0] > DETECTION_TH:
            if classes[0][0] in [1, 2, 3]:
                return classes[0][0] - 1
        return -1

