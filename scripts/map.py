#!/usr/bin/python3

import rospy
import tf
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def loadMap(path: str)-> dict:
    bases_coord = dict()
    with open(path, "r") as f:
        lines = f.readlines()
        for line_raw in lines[1:]:
            line = line_raw.split()
            bases_coord[line[0]] = [float(i) for i in line[1:]]
    return bases_coord

def msg_marker(bases_coord: dict) -> list:
    array = []
    for base in bases_coord.keys():
        msg = Marker()
        msg.type = 3
        msg.ns = "anchor" + base
        msg.header.frame_id = "anchor_map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = bases_coord[base][0]
        msg.pose.position.y = bases_coord[base][1]
        msg.pose.position.z = bases_coord[base][2]

        # quaternion = quaternion_from_euler([bases_coord[base][3], bases_coord[base][4], bases_coord[base][5]])
        # quaternion = quaternion_from_euler([0, 0, 0])
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        msg.color.a = 1.0

        msg.scale.x = 0.5
        msg.scale.y = 0.5
        msg.scale.z = 0.1
        
        
        array.append(msg)
    return array

def main():
    path_map = rospy.get_param("/map")

    pub = rospy.Publisher("/dwm1000/visualization_anchor", MarkerArray, queue_size=10)
    rospy.init_node("anchor_map")
    pub_msg = MarkerArray()
    coord = loadMap(path_map)
    msg_list = msg_marker(coord)
    pub_msg.markers = msg_list

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "anchor_map",
                         "map")
        pub.publish(pub_msg)
        rate.sleep()

if __name__ == "__main__":
    main()