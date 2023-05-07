import rospy
from scipy import signal
import math
import numpy as np

from dwm1000_msgs.msg import BeaconDataArray
from dwm1000_msgs.msg import BeaconData


def callback(data):
    global pub, msg_list, distances_raw, beacons_zi, b, a
    distances = {}
    for beacon in data.beacons:
        distances_raw[beacon.id] = beacon.dist

    "filter"
    for key in list(distances_raw.keys()):
        distances[key], beacons_zi[str(key)] = signal.lfilter(
            b, a, [distances_raw.get(key)], zi=beacons_zi[str(key)])

    pub_msg = BeaconDataArray()
    for key in list(distances.keys()):
        msg = BeaconData()
        msg.id = int(key)
        msg.dist = float(distances.get(key))
        msg_list.append(msg)
    pub_msg.beacons = msg_list
    pub.publish(pub_msg)
    msg_list.clear()
    distances_raw.clear()


if __name__ == "__main__":
    global pub, msg_list, distances_raw, beacons, b, a

    rospy.init_node("filter", anonymous=True)

    beacons = rospy.get_param("/bases_coord")
    beacons_zi = {}
    msg_list = []
    distances_raw = {}
    b, a = signal.butter(25, 0.1)
    for key in list(beacons.keys()):
        beacons_zi[key] = signal.lfilter_zi(b, 1)

    pub = rospy.Publisher("dwm1000/beacon_data_filter",
                          BeaconDataArray, queue_size=10)
    rospy.Subscriber("dwm1000/beacon_data", BeaconDataArray, callback)
    rospy.spin()
