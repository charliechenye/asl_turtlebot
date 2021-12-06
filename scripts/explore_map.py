#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from time import sleep


class PublishWayPoint:
    def __init__(self):
        rospy.init_node("turtlebot_waypoint", anonymous=False)
        self.switch_sub = rospy.Subscriber('/retrieve_next_waypoint', Bool, self.retrieve_next_way_point)

        self.way_point_lst_pub = rospy.Publisher("/cmd_nav", Pose2D, queue_size=10)
        self.way_point_viz_pub = rospy.Publisher('/marker_way_point', Marker, queue_size=10)

        self.delayed_publish_exp = rospy.get_param("~delay_publish_explore", 1)
        self.delayed_publish_res = rospy.get_param("~delay_publish_rescue", 3)
        self.marker_size = rospy.get_param("~marker_size", 0.1)

        self.explore_phase = True
        self.en_route_rescue = True
        self.delayed_publish = self.delayed_publish_exp

        self.published_i = 0
        self.way_point_list = []
        self.way_point_viz = []
        self.location_point_list = []
        self.location_point_vis = []
        # Preset Way Points
        s = """
        x: 3.4467575777903927
        y: 2.7978498968388306
        theta: 3.103931069002174
        =============== 0
        x: 2.4358298414390274
        y: 2.8034369370385788
        theta: 3.111662310575331
        =============== 1
        x: 1.0433522377214772
        y: 2.8456794451402136
        theta: 3.1188173344552887
        =============== 2
        x: 0.4950856461222242
        y: 2.7003305802671758
        theta: -2.515492692466538
        =============== 3
        x: 0.3701785282398169
        y: 1.98525163338255
        theta: -1.6067560787628778
        =============== 4
        x: 0.31693564742110025
        y: 1.7159908495908391
        theta: -1.5753459265220457
        =============== 5
        x: 0.9824716271298932
        y: 1.653738937686387
        theta: -0.02766161224616703
        =============== 6
        x: 0.2873089552660169
        y: 0.42930977308234314
        theta: -0.07321943619324434
        =============== 7
        x: 3.055004399253292
        y: 0.28138946165828044
        theta: 0.04920500856623527
        =============== 8
        x: 3.10337555110641
        y: 1.4222088105199289
        theta: 1.4905407683116543
        =============== 9
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

        self.retrieve_next_way_point(Bool(True))

    def retrieve_next_way_point(self, msg):
        if not msg.data:
            rospy.loginfo("ERROR!!!!")
            return
        if self.explore_phase:
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
                self.explore_phase = False
                self.delayed_publish = self.delayed_publish_res
                self.published_i = 0
        if not self.explore_phase and self.published_i < self.total_waypoints - 1:
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

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    wp = PublishWayPoint()
    wp.run()
