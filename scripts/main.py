#!/usr/bin/ python3
from trilateration import Trilateration
from trilateration.msg import BeaconDataArra
from trilateration.msg import Point3D

bases_coord = { "1": [0, 0, 0],
                "2": [2, 0, 0],
                "3": [0, 2, 0],
                "4": [-2, -2, 0.2],
                "5": [2, 2, 0.2],
                "6": [2, -2, 0],
                "7": [-2, 2, 0.2]
}

def callback(data):
    global pub, tril, x0
    distances = dist()
    for in data.beacons:
        


def main():

    tril = Trilateration(bases_coord)

    # setting the starting position
    x0 = np.zeros([len(list(bases_coord.values())[0])])

    pub = rospy.Publisher('Point3D', Point3D, queue_size=10)
    rospy.init_node('trilateration', anonymous=True)
    rospy.Subscriber("BeaconData", BeaconDataArra, callback)
    rospy.spin()

if __name__ == "__main__":
    main()