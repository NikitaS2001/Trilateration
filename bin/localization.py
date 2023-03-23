#!/usr/bin/python3

import scipy
from scipy.optimize import root
import math
import rospy
import numpy as np

from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from dwm1000_msgs.msg import BeaconDataArray
from trilateration import Trilateration

bases_coord = {0: [0, 0, 0],
               1: [1.45, 1.45, 0.28],
               2: [0, 1.45, 0],
               3: [1.45, 0, 0]
               }

offset = {0: -0.3,
          1: -0.3,
          2: -0.3,
          3: -0.3
          }

def correct_dist(distances: dict, offset: dict):
    """ Return dict

    adding offset to the resulting distances
    """
    for base in list(distances.keys()):
        distances[base] += offset[base]

    return distances


def callback(data):
    global pub, tril
    distances = dict()
    for beacon in data.beacons:
        distances[beacon.id] = beacon.dist

    distances = correct_dist(distances, offset)

    # trilateration solution
    sol = tril.solve(distances, method="lm")
    # rospy.loginfo(sol)

    point3d = np.array([sol.x[i] for i in range(3)], dtype=np.float)
    point3d = np.around(point3d, decimals=4)
    rospy.loginfo(point3d * 100)

    msg = Marker()

    msg.header.frame_id = "base_link"
    msg.header.stamp = rospy.Time()
    msg.ns = "test"
    msg.type = Marker.SPHERE
    msg.action = Marker.ADD
    msg.id = 0
    msg.pose.position.x = point3d[0]
    msg.pose.position.y = point3d[1]
    msg.pose.position.z = point3d[2]
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    msg.pose.orientation.w = 1
    msg.scale.x = 0.05
    msg.scale.y = 0.05
    msg.scale.z = 0.05
    msg.color.a = 1
    msg.color.r = 1
    msg.color.g = 0
    msg.color.b = 0

    pub.publish(msg)


def main():
    global tril, pub
    # setting the starting position
    x0 = np.zeros([len(list(bases_coord.values())[0])])
    tril = Trilateration(bases_coord, x0)

    pub = rospy.Publisher("dwm1000/point3d", Marker, queue_size=10)
    rospy.init_node("trilateration", anonymous=True)
    rospy.Subscriber("dwm1000/beacon_data", BeaconDataArray, callback)
    rospy.spin()


if __name__ == "__main__":
    main()
