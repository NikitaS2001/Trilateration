import rospy
from scipy import signal
import math
import numpy as np

from dwm1000_msgs.msg import BeaconDataArray
from dwm1000_msgs.msg import BeaconData


def callback(data):
    global pub, msg_list, distances_raw
    for beacon in data.beacons:
        distances_raw[beacon.id] = beacon.dist

    distances = distances_raw

    "filter"

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
    global pub, msg_list, distances_raw

    rospy.init_node("filter", anonymous=True)

    msg_list = []
    distances_raw = {}
    a, b = signal.butter(3, 2*1.25/5)
    filter = signal.filtfilt(a, b)
    buffer = np.zeros([5])

    pub = rospy.Publisher("dwm1000/beacon_data_filter",
                          BeaconDataArray, queue_size=10)
    rospy.Subscriber("dwm1000/beacon_data", BeaconDataArray, callback)
    rospy.spin()
