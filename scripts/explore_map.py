#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from time import sleep


class PublishWayPoint:
    def __init__(self):
        rospy.init_node("turtlebot_waypoint", anonymous=False)
        rospy.Subscriber('/retrieve_next_waypoint', Bool, self.retrieve_next_way_point)

        # self.obj_sub = rospy.Subscriber('/object_locations', Pose2D, self.record_object)

        self.way_point_lst_pub = rospy.Publisher("/cmd_nav", Pose2D, queue_size=10)
        self.way_point_viz_pub = rospy.Publisher('/marker_way_point', Marker, queue_size=10)
        self.switch_to_rescue_pub = rospy.Publisher('/cmd_switch_rescue', Bool, queue_size=10)

        self.delayed_publish_exp = rospy.get_param("~delay_publish_explore", 1)
        self.delayed_publish_res = rospy.get_param("~delay_publish_rescue", 1)
        self.marker_size = rospy.get_param("~marker_size", 0.1)

        self.explore_phase = True
        self.reverse_order = True
        self.delayed_publish = self.delayed_publish_exp

        self.way_point_list = []
        self.way_point_viz = []
        self.location_point_list = []
        self.location_point_vis = []
        # Preset Way Points
        s = """
x: 3.321861347827481
y: 2.725953261607724
theta: 3.078438094291383
        =============== 0
x: 2.4342121552642415
y: 2.779541107321901
theta: 3.1195350101739536
        =============== 1
x: 1.0037986326066983
y: 2.792201596820265
theta: -3.1216916622446425
        =============== 2
x: 0.5518598105268782
y: 2.6946314602746373
theta: -2.311016170992066
        =============== 3
x: 0.3163584054551258
y: 2.094468408984719
theta: -1.5938943547681368
        =============== 4
x: 0.30302747257331214
y: 1.6173143394882556
theta: -0.0399425261345982
        =============== 5
x: 0.6321473680387281
y: 1.6173143394882556
theta: -0.03290439356208351
        =============== 6
x: 0.30302747257331214
y: 1.6173143394882556
theta: -0.0399425261345982
        =============== 7
x: 0.2598550167936355
y: 0.39144526584594375
theta: -0.02852933915593895
        =============== 8
x: 3.0935541791441286
y: 0.29994736960875135
theta: 0.02410869149263058
        =============== 9
x: 3.156873414822996
y: 1.3148665367085326
theta: 1.6007259330564694
        =============== 10
        """
        counter = 0
        x = y = th = 0.0
        for line in s.splitlines():
            if not line.strip():
                continue
            if counter == 0:
                x = float(line.split(':')[1].strip())
            elif counter == 1:
                y = float(line.split(':')[1].strip())
            elif counter == 2:
                th = float(line.split(':')[1].strip())
            else:
                self.way_point_list.append(Pose2D(x, y, th))
                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = 0
                marker.type = 2  # sphere

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size

                marker.color.a = 0.5  # Don't forget to set the alpha!
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                self.way_point_viz.append(marker)

            counter = (counter + 1) % 4
        self.total_waypoints = len(self.way_point_list)
        self.total_locations = 0

        if self.reverse_order:
            self.published_i = self.total_waypoints - 1
        else:
            self.published_i = 0

        self.retrieve_next_way_point(Bool(True))
        self.switch_to_rescue_pub.publish(Bool(False))

    def retrieve_next_way_point(self, msg):
        if not msg.data:
            rospy.loginfo("ERROR!!!!")
            return
        if self.explore_phase:
            if self.reverse_order:
                if self.published_i >= -1:
                    rospy.loginfo("Received Instruction to publish waypoint %d" % self.published_i)
                    sleep(self.delayed_publish)
                    rospy.loginfo("Publishing waypoint %d" % self.published_i)
                    self.way_point_viz[self.published_i].header.stamp = rospy.Time()
                    old_pose_2d = self.way_point_list[self.published_i]
                    reverse_pose_2d = Pose2D(old_pose_2d.x, old_pose_2d.y, -old_pose_2d.theta)
                    self.way_point_viz_pub.publish(self.way_point_viz[self.published_i])
                    self.way_point_lst_pub.publish(self.way_point_list[self.published_i])
                    self.published_i -= 1
                else:
                    rospy.loginfo("No more way points to explore. Ready to switch to rescue mode")
                    self.switch_to_rescue_pub.publish(Bool(True))
                    self.explore_phase = False
                    self.delayed_publish = self.delayed_publish_res
                    self.published_i = 0
            else:
                if self.published_i < self.total_waypoints:
                    rospy.loginfo("Received Instruction to publish waypoint %d" % self.published_i)
                    sleep(self.delayed_publish)
                    rospy.loginfo("Publishing waypoint %d" % self.published_i)
                    self.way_point_viz[self.published_i].header.stamp = rospy.Time()
                    self.way_point_viz_pub.publish(self.way_point_viz[self.published_i])
                    self.way_point_lst_pub.publish(self.way_point_list[self.published_i])
                    self.published_i += 1
                else:
                    rospy.loginfo("No more way points to explore. Ready to switch to rescue mode")
                    self.switch_to_rescue_pub.publish(Bool(True))
                    self.explore_phase = False
                    self.delayed_publish = self.delayed_publish_res
                    self.published_i = 0

        """
        if not self.explore_phase and self.rescue_waypoint_j < self.total_waypoints - 1:
            # try rescue navigation
            if self.en_route_rescue:
                rospy.loginfo("Received Instruction to publish waypoint %d" % self.published_i)
                sleep(self.delayed_publish)
                rospy.loginfo("Publishing waypoint %d" % self.published_i)
                self.way_point_viz[self.published_i].header.stamp = rospy.Time()
                self.way_point_viz_pub.publish(self.way_point_viz[self.published_i])
                self.way_point_lst_pub.publish(self.way_point_list[self.published_i])
                self.published_i += 1
                self.en_route_rescue = False
            else:
                rospy.loginfo("Received Instruction to home waypoint")
                sleep(self.delayed_publish)
                rospy.loginfo("Publishing home address")
                self.way_point_viz[self.published_i].header.stamp = rospy.Time()
                self.way_point_viz_pub.publish(self.way_point_viz[-1])
                self.way_point_lst_pub.publish(self.way_point_list[-1])
                self.en_route_rescue = True
        """

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    wp = PublishWayPoint()
    wp.run()