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
        self.delayed_publish = self.delayed_publish_exp

        self.published_i = 0
        self.way_point_list = []
        self.way_point_viz = []
        self.location_point_list = []
        self.location_point_vis = []
        # Preset Way Points
        s = """
        x: 3.3801320611055896
        y: 2.745296531680954
        theta: 3.0570938410810156
        =============== 0
        x: 0.8034759717197456
        y: 2.8012834410845753
        theta: 3.066100508970252
        =============== 1
        x: 0.3357249527689034
        y: 2.2638362881155616
        theta: -1.5922447311998582
        =============== 2
        x: 0.29876760085549764
        y: 1.7453475098065703
        theta: -1.6526198100684875
        =============== 3
        x: 1.049586688519263
        y: 1.6652466670640682
        theta: -0.019619629810605655
        =============== 4
        x: 0.22703147912742788
        y: 0.4704677098883807
        theta: -0.014436819315202038
        =============== 5
        x: 3.0285354617515936
        y: 0.3191914797586132
        theta: -0.048871279185151155
        =============== 6
        x: 3.0564035452123277
        y: 1.380957798731946
        theta: 1.5578916364120823
        =============== 7
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
