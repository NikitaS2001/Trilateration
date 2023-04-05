import rospy

from dwm1000_msgs.msg import BeaconDataArray
from sensor_msgs.msg import Range

def callback(data):
    global pub

    msg_pub = Range()

    msg_pub.header.stamp = rospy.Time().now()
    msg_pub.range = data.beacons[0].dist

    pub.publish(msg_pub)



def main():
    global msg_pub, pub
    pub = rospy.Publisher("dwm1000/beacon_range", Range, queue_size=10)
    
    rospy.init_node("pub_range_beacon", anonymous=True)

    while not rospy.is_shutdown():

        rospy.Subscriber("dwm1000/beacon_data", BeaconDataArray, callback)
        rospy.spin()

if __name__ == "__main__":
    main()