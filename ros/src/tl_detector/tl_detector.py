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
from scipy import spatial

STATE_COUNT_THRESHOLD = 3

GT_ENABLED = True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.light = 0
        self.light_waypoints_list = None
        self.tree = None

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

        self.stop_line_positions = self.config['stop_line_positions']

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.light_waypoints_list = self.get_stop_lines_waypoints()

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
            rospy.loginfo('State of the Traffic light: {}'.format(self.state))
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        car_x = pose.position.x
        car_y = pose.position.y
        car_w = pose.position.w
        car_theta = 2 * np.arccos(car_w)

        if car_theta > np.pi:
            car_theta = -(2 * np.pi - car_theta)

        if self.tree is None:
            waypoint_list = []
            for index, waypoint in enumerate(self.waypoints.waypoints):
                x = waypoint.pose.pose.position.x
                y = waypoint.pose.pose.position.y
                waypoint_list.append([x, y])
            self.tree = spatial.KDTree(np.asarray(waypoint_list))

        dist, wp_ind = self.tree.query([pose.position.x, pose.position.y])

        wp_x = self.waypoints.waypoints[wp_ind].pose.pose.position.x
        wp_y = self.waypoints.waypoints[wp_ind].pose.pose.position.y

        head = np.arctan2((wp_y - car_y), (wp_x - car_x))
        angle = abs(car_theta - head)
        if angle > (np.pi /4):
            wp_ind = (wp_ind + 1) % len(self.waypoints.waypoints)
            
        return wp_ind

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(GT_ENABLED):
            return self.lights[light].state
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        #stop_line_positions = self.config['stop_line_positions']
        #if(self.pose):
        #    car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        #if light:
        #    state = self.get_light_state(light)
        #    return light_wp, state
        #self.waypoints = None

        if (self.light_waypoints_list != None and self.pose != None):
            max_light_waypoint = self.light_waypoints_list[-1]["wp"]
            closest_waypoint = self.get_closest_waypoint(self.pose.pose)
            light_waypoint = self.light_waypoints_list[self.light]["wp"]

            if (closest_waypoint > light_waypoint):
                if (self.light != 0 or closest_waypoint < max_light_waypoint):
                    self.light = (self.light + 1) % len(self.light_waypoints_list)
                    light_waypoint = self.light_waypoints_list[self.light]["wp"]

            state = self.get_light_state(self.light)
            return light_waypoint, state
        else:
            return -1, TrafficLight.UNKNOWN

    def get_stop_lines_waypoints(self):
        stop_lines_waypoints_list = []

        for i in range(len(self.stop_line_positions)):
            stop_line = Pose()
            stop_line.position.x = self.stop_line_positions[i][0]
            stop_line.position.y = self.stop_line_positions[i][1]

            closest_light_waypoint = self.get_closest_waypoint(stop_line)

            waypoint_dict = {'ind':i, 'waypoint':closest_light_waypoint}

            stop_lines_waypoints_list.append(d)

            sorted_waypoints_list = sorted(stop_lines_waypoints_list, key=lambda k: k['wp'])

            return sorted_list
        


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
