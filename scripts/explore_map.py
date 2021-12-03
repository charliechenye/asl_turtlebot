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

        self.delayed_publish = rospy.get_param("~delay_publish", 3)
        self.marker_size = rospy.get_param("~marker_size", 0.1)

        self.explore_phase = True

        self.published_i = 0
        self.way_point_list = []
        self.way_point_viz = []
        self.location_point_list = []
        self.location_point_vis = []
        # Preset Way Points
        s = """
        x: 3.336607942302636
        y: 2.819031940252024
        theta: -3.1337109176308444
        =============== 0
        x: 1.4704580499488753
        y: 2.8396105977631745
        theta: -3.1345847455902196
        =============== 1
        x: 0.6542604142070271
        y: 2.7602622759589486
        theta: -2.241586118886368
        =============== 2
        x: 0.31964809544737827
        y: 2.40314431516008
        theta: -1.548087302037842
        =============== 3
        x: 0.3228542879172513
        y: 1.663114566595889
        theta: -1.599865705849584
        =============== 4
        x: 1.0393564713035321
        y: 1.6694369047303752
        theta: 0.041272942007544944
        =============== 5
        x: 0.3228542879172513
        y: 0.37771431672149214
        theta: -0.06504570807514945
        =============== 6
        x: 2.298943537965215
        y: 0.3213881756165929
        theta: -0.030613244491231987
        =============== 7
        x: 3.0280892038908633
        y: 0.3594165254020387
        theta: -0.04109991810864817
        =============== 8
        x: 3.027884373979012
        y: 1.3971543957411448
        theta: 1.5357870415504642
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


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    wp = PublishWayPoint()
    wp.run()
