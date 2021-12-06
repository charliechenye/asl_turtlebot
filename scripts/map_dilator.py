#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from scipy.signal import correlate2d
import numpy as np

class DilateMap:
    def __init__(self):
        rospy.init_node("map_dilator", anonymous=False)
        self.map_in_sub = rospy.Subscriber("/map", OccupancyGrid, self.dilate_map)
        self.dilated_map_pub = rospy.Publisher("/map_dilated", OccupancyGrid, queue_size=10)
        self.width = self.height = None

        robot_size = rospy.get_param("~robot_size", 6)
        self.correlation_filter = np.ones((robot_size, robot_size))

        self.OCCUPIED = 100
        self.FREE = 0
        self.UNKNOWN = -1
        self.DILATE_PROB = rospy.get_param("~dilate_probability", 30)

    def dilate_map(self, msg):
        output_map = OccupancyGrid()
        output_map.header = msg.header
        output_map.info = msg.info
        if self.width is None:
            self.width, self.height = msg.info.width, msg.info.height

        old_map = np.array(msg.data)
        new_map = old_map.reshape((self.height, self.width))
        print(new_map)
        new_map[new_map < 0] = 0
        print(new_map)
        new_map = correlate2d(new_map, self.correlation_filter, mode='same')
        print(new_map)
        new_map[new_map > 0] = self.DILATE_PROB
        print(new_map)
        new_map = new_map.flatten()
        new_map[old_map == self.OCCUPIED] = self.OCCUPIED
        print(new_map)
        new_map[np.array(msg.data) < 0] = self.UNKNOWN
        print(new_map)
        output_map.data = new_map.astype(int).tolist()

        self.dilated_map_pub.publish(output_map)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    wp = DilateMap()
    wp.run()
