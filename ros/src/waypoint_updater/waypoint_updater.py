#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.act_velocity = None
        self.brake_range = []
        self.on_brake = False
        
        self.base_waypoints = None
        self.next_waypoint = None
        self.last_stopline_waypoint = -1
        self.waypoint_velocity = self.get_vel_in_mps(rospy.get_param('/waypoint_loader/velocity'))

        self.tree = None

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

        #rospy.spin()

    def self.get_vel_in_mps(self, velocity):
        return (velocity * 1000.) / (60. * 60.)

    def self.get_next_waypoint_index():
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        car_w = self.current_pose.orientation.w

        car_theta = 2 * np.arccos(car_w)

        if car_theta > np.pi:
            car_theta = -(2 * np.pi - car_theta)

        if self.tree is None:
            waypoint_list = []
            for index, waypoint in enumerate(self.base_waypoints):
                x = waypoint.pose.pose.position.x
                y = waypoint.pose.pose.position.y
                waypoint_list.append([x, y])
            self.tree = spatial.KDTree(np.asarray(waypoint_list))

        dist, wp_ind = self.tree.query([self.current_pose.position.x, self.current_pose.position.y])

        wp_x = self.base_waypoints[wp_ind].pose.pose.position.x
        wp_y = self.base_waypoints[wp_ind].pose.pose.position.y

        head = np.arctan2((wp_y - car_y), (wp_x - car_x))
        angle = abs(car_theta - head)
        if angle > (np.pi /4):
            wp_ind = (wp_ind + 1) % len(self.base_waypoints)
            
        return wp_ind

    def publish_waypoints(self, next_waypoint):
        msg = Lane()
        msg.waypoints = []
        for i in range(LOOKAHEAD_WPS):
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = self.base_waypoints[next_waypoint].pose.pose.position.x
            waypoint.pose.pose.position.y = self.base_waypoints[next_waypoint].pose.pose.position.y
            waypoint.twist.twist.linear.x = self.base_waypoints[next_waypoint].twist.twis.linear.x
            msg.waypoints.append(waypoint)
            #Verify
            next_waypoint = (next_waypoint + 1) % len(self.base_waypoints)

        self.final_waypoints_pub.publish(msg)
    
    def loop(self):
        if(self.current_pose is not None) and (self.base_waypoints is not None):
            self.next_waypoint = self.get_next_waypoint_index()
            self.publish_waypoints(self.next_waypoint)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if (self.base_waypoints is not None and self.act_velocity != None):
            waypoint_index = msg.data

            if (waypoint_index != -1 and self.on_brake == False):
                dist = self.distance(self.base_waypoints, self.next_waypoint, waypoint_index)

                index_distance = waypoint_index - self.next_waypoint
                s_per_idx = dist/index_distance if index_distance else 0.9

                ref_acceleration = (self.act_velocity**2)/(2*(dist-4))

                if(ref_acceleration >= 2 and ref_acceleration < 10):

                    for i in range(self.next_waypoint, waypoint_index + 1):
                        updated_velocity = self.act_velocity**2 - 2*ref_acceleration*s_per_idx*(i-self.next_waypoint+1)
                        updated_velocity = np.sqrt(updated_velocity) if (updated_velocity >= 0.0) else 0

                        self.set_waypoint_velocity(self.base_waypoints, i, updated_velocity)

                    self.brake_range.append((self.next_waypoint, waypoint_index))
                    self.on_brake = True

                else:
                    pass

            if (waypoint_index == -1 and self.last_stopline_waypoint != -1):
                for i in self.brake_range:
                    for j in range(i[0], i[1]+1):
                        self.set_waypoint_velocity(self.base_waypoints, j, self.waypoint_velocity)
                    self.brake_range.remove(i)
                    self.on_brake = False

            self.last_stopline_waypoint = waypoint_index

            

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def velocity_cb(self, msg):
        self.act_velocity = msg.twist.linear.x

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
