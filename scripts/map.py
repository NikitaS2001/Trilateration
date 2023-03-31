#!/usr/bin/python3

from os import path
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

map_path = "/home/futureskills2/catkin_ws/src/dwm1000_pose/map/map.txt"


def main():
    pub = rospy.Publisher("dwm1000/visualization/anchors",
                          MarkerArray, queue_size=10)
    rospy.init_node("visualization_anchors", anonymous=True)
    rate = rospy.Rate(10)

    msg = Marker()
    pub_msg = MarkerArray()
    f = open(path, "r")
    for line in f:
        print(line)
    f.close()

    while not rospy.is_shutdown():

        pub.publish(pub_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
