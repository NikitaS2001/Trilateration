#!/usr/bin/python3

import rospy
import tf
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def loadMap(path: str)-> dict:
    bases_coord = {}
    with open(path, "r") as f:
        lines = f.readlines()
        for line_raw in lines[1:]:
            line = line_raw.split()
            bases_coord[line[0]] = [float(i) for i in line[1:]]
    rospy.set_param("/bases_coord", bases_coord)
    return bases_coord

def sendTf_anchor(bases_coord: dict,
                  Broadcaster: tf.TransformBroadcaster) -> list:
    for base in bases_coord.keys():
        Broadcaster.sendTransform((bases_coord[base][0], bases_coord[base][1], bases_coord[base][2]),
                                  quaternion_from_euler(0, 0, 0, axes="rxyz"),
                                  rospy.Time.now(),
                                  "anchor"+base,
                                  "anchor_map")

def sendTf_map(Broadcaster: tf.TransformBroadcaster):
    Broadcaster.sendTransform((0.0, 0.0, 0.0),
                         quaternion_from_euler(0, 0, 0, axes="rxyz"),
                         rospy.Time.now(),
                         "anchor_map",
                         "map")

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

        quaternion = quaternion_from_euler(ai=bases_coord[base][3],
                                           aj=bases_coord[base][4],
                                           ak=bases_coord[base][5],
                                           axes='rzyx')
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        msg.color.a = 1.0

        msg.scale.x = 0.3
        msg.scale.y = 0.3
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
        sendTf_map(br)
        sendTf_anchor(coord, br)
        pub.publish(pub_msg)
        rate.sleep()

if __name__ == "__main__":
    main()