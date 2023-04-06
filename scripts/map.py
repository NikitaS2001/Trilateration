#!/usr/bin/python3

import rospy
import tf

def main():
    map = rospy.get_param("/map")
    rospy.init_node("anchor_map")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "anchor_map",
                         "map")
        rate.sleep()

if __name__ == "__main__":
    main()