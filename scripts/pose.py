#!/usr/bin/python3

import rospy
import math
import numpy as np
from scipy.optimize import root

import tf2_ros
import tf2_geometry_msgs

from dwm1000_msgs.msg import BeaconDataArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

bases_coord = {0: [3, 0, 0.07],
               1: [0, 3, 0.07],
               2: [0, 0, 0.07+0.72],
               3: [3, 3, 0.07]
               #4: [-1.25, 0, 0]
               }

offset = {0: -0.35,
          1: -0.35,
          2: -0.35,
          3: -0.35
          }

class Trilateration:

    """Solution of the trilateration problem
    in the form of a system of nonlinear equations
    using global optimization methods"""

    __bases_coord: dict
    __base_dist: dict
    __tolerance: float = 5e-4  # tolerance for termination
    __iterMax: int = 20  # maximum number of iterations
    # previous solution (default value [0, 0, 0])
    __prev_sol: np.ndarray


    def __init__(self, base_coord: dict, prev_sol: np.ndarray = np.zeros([3])) -> None:
        self.__bases_coord = base_coord
        self.__prev_sol = prev_sol

    def set_bases_coord(self, base_coord: dict) -> None:
        self.__bases_coord = base_coord

    def get_bases_coord(self) -> dict:
        return self.__bases_coord

    def set_iterMax(self, iterMax: int) -> None:
        self.__iterMax = iterMax

    def get_iterMax(self) -> int:
        return self.__iterMax

    def solve(self, base_dist: dict, method="3D"):

        self.__base_dist = base_dist

        if method == "3D":
            sol = self.__solvelm3D()
            self.__prev_sol = np.array(sol.x)
            return sol
        elif method == "2D":
            sol = self.__solvelm2D()
            self.__prev_sol = np.array(sol.x)
            return sol


    def __solvelm3D(self):
        """ Return a OptimizeResult 3D (x, y, z)

        Solves a system of nonlinear equations using the least squares method
        using the Levenberg-Marquardt algorithm
        """
        return root(self.__fun3D,
                    self.__prev_sol,
                    jac=self.__jac3D,
                    method="lm",
                    options={"col_deriv": False,
                             "xtol": self.__tolerance,
                             "maxiter": self.__iterMax}
                    )

    def __jac3D(self, v):
        """ Return function
        bases_coord
        Creation of the Jacobian matrix
        """
        f = np.zeros([len(self.__base_dist.keys()), len(
            list(self.__bases_coord.values())[0])])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(len(list(self.__bases_coord.values())[0])):
                f[i][j] = 2*(v[j]-self.__bases_coord.get(base)[j])
        return f

    def __fun3D(self, v):
        """ Return function

        Creation of a system of nonlinear equations        
        """
        f = np.zeros([len(self.__base_dist)])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(len(list(self.__bases_coord.values())[0])):
                f[i] += math.pow(v[j]-(self.__bases_coord.get(base)[j]), 2)
            f[i] -= math.pow(self.__base_dist.get(base), 2)
        return f

    def __solvelm2D(self):
        """ Return a OptimizeResult 2D (x, y)

        Solves a system of nonlinear equations using the least squares method
        using the Levenberg-Marquardt algorithm
        """
        return root(self.__fun2D,
                    self.__prev_sol,
                    jac=self.__jac2D,
                    method="lm",
                    options={"col_deriv": False,
                             "xtol": self.__tolerance,
                             "maxiter": self.__iterMax}
                    )

    def __jac2D(self, v):
        """ Return function
        bases_coord
        Creation of the Jacobian matrix
        """
        pass

    def __fun2D(self, v):
        """ Return function

        Creation of a system of nonlinear equations        
        """
        pass

def correct_dist(distances: dict, offset: dict):
    """ Return dict

    adding offset to the resulting distances
    """
    for base in list(distances.keys()):
        distances[base] += offset[base]

    return distances

def range_callback(data):
    global z_range

    z_range = data.range



def callback(data):
    global pub, tril, z_range, tf_buffer, tf_listener
    distances = dict()
    for beacon in data.beacons:
        distances[beacon.id] = beacon.dist

    # distances = correct_dist(distances, offset)

    # trilateration solution
    sol = tril.solve(distances, method="3D")
    # rospy.loginfo(sol)

    point3d = np.array([sol.x[i] for i in range(3)])
    point3d = np.around(point3d, decimals=4)
    rospy.loginfo(point3d * 100)
    
    msg = PoseStamped()

    msg.header.frame_id = "anchor_map"
    msg.header.stamp = rospy.Time().now()
    msg.pose.position.x = point3d[0]
    msg.pose.position.y = point3d[1]
    msg.pose.position.z = z_range if z_range is not None else point3d[2]
    # msg.pose.orientation.x = 0
    # msg.pose.orientation.y = 0
    # msg.pose.orientation.z = 0
    # msg.pose.orientation.w = 1

    # try:
    #     transform = tf_buffer.lookup_transform(
    #         "map",
    #         msg.header.frame_id,
    #         msg.header.stamp,
    #         rospy.Duration(1.0))

    #     msg = tf2_geometry_msgs.do_transform_pose(
    #         msg,
    #         transform)

    #     pub.publish(msg)

    # except Exception as e:
    #     print(f"Could not find transform to map! {e}")

    pub.publish(msg)

def main():
    rospy.init_node("trilateration", anonymous=True)

    global tril, pub, z_range, tf_buffer, tf_listener
    # setting the starting position
    x0 = np.zeros([len(list(bases_coord.values())[0])])
    # x0 = np.array([1.45/2, 1.45/2, 0.07])
    z_range = None
    tril = Trilateration(bases_coord, x0)


    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)

    rospy.Subscriber("dwm1000/beacon_data", BeaconDataArray, callback)
    rospy.Subscriber("rangefinder/range", Range, range_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
