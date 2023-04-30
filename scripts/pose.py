#!/usr/bin/python3

import rospy
import math
import numpy as np
from scipy.optimize import root

from dwm1000_msgs.msg import BeaconDataArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

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
    __height: float = None

    def __init__(self, base_coord: dict, prev_sol: np.ndarray = np.zeros([3]), offset: dict = None ) -> None:
        self.__bases_coord = base_coord
        self.__prev_sol = prev_sol
        self.__offset = offset

        # initialization node
        rospy.init_node("trilateration", anonymous=True)

        self.__pub = rospy.Publisher("mavros/vision_pose/pose",
                                     PoseStamped, queue_size=10)

        rospy.Subscriber("dwm1000/beacon_data",
                         BeaconDataArray, self.__callback)
        rospy.Subscriber("rangefinder/range", Range, self.__height_callback)

    def set_bases_coord(self, base_coord: dict) -> None:
        self.__bases_coord = base_coord

    def get_bases_coord(self) -> dict:
        return self.__bases_coord

    def set_iterMax(self, iterMax: int) -> None:
        self.__iterMax = iterMax

    def get_iterMax(self) -> int:
        return self.__iterMax

    def set__offset(self, offset: dict) -> None:
        "adding offset to the resulting distances"
        self.__offset = offset

    def get_offset(self) -> dict:
        return self.__offset

    def __height_callback(self, data):
        self.__height = data.range

    def __correct_dist(self, distances: dict):
        """ Return dict

        adding offset to the resulting distances
        """

        for base in list(distances.keys()):
            distances[base] += self.__offset[base]

        return distances

    def __callback(self, data):
        distances = {}
        for beacon in data.beacons:
            distances[beacon.id] = beacon.dist

        # adding offset to the resulting distances
        if offset != None:
            distances = self.__correct_dist(distances)

        sol = self.solve(distances, method="3D")

        point = np.array([sol.x[i] for i in range(3)])
        point = np.around(point, decimals=4)

        rospy.loginfo(point * 100)

        msg = PoseStamped()
        msg.header.frame_id = "anchor_map"
        msg.header.stamp = rospy.Time().now()
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.position.z = self.__height if self.__height is not None else point[2]

        self.__pub.publish(msg)

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

        Creation of the Jacobian matrix
        """
        f = np.zeros([len(self.__base_dist.keys()), len(
            list(self.__bases_coord.values())[0])])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(4):
                f[i][j] = 2*(v[j]-self.__bases_coord.get(str(base))[j])
        return f

    def __fun3D(self, v):
        """ Return function

        Creation of a system of nonlinear equations
        """
        f = np.zeros([len(self.__base_dist)])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(4):
                f[i] += math.pow(v[j] -
                                 (self.__bases_coord.get(str(base))[j]), 2)
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
        f = np.zeros([len(self.__base_dist.keys()), 2])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(3):
                f[i][j] = 2*(v[j]-self.__bases_coord.get(str(base))[j])
        return f

    def __fun2D(self, v):
        """ Return function

        Creation of a system of nonlinear equations
        """
        f = np.zeros([len(self.__base_dist)])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(3):
                f[i] += math.pow(v[j] -
                                 (self.__bases_coord.get(str(base))[j]), 2)
            f[i] += math.pow(self.__height -
                             (self.__bases_coord.get(str(base))[j]), 2)
            f[i] -= math.pow(self.__base_dist.get(base), 2)
        return f


def main():

    offset = {0: -0.35,
            1: -0.35,
            2: -0.35,
            3: -0.35
            }

    bases_coord = None
    while not bool(bases_coord):
        try:
            bases_coord = rospy.get_param("/bases_coord")
        except:
            pass

    x0 = np.zeros([len(list(bases_coord.values())[0])])
    # x0 = np.zeros([2])
    tril = Trilateration(bases_coord, x0)
    rospy.spin()


if __name__ == "__main__":
    main()
