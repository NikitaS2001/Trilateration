#!/usr/bin/python3

import scipy
from scipy.optimize import root
import math
import rospy
import numpy as np

from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker
from dwm1000_msgs.msg import BeaconDataArray

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

    def solve(self, base_dist: dict, method="lm"):

        self.__base_dist = base_dist

        if method == "lm":
            sol = self.__solvelm()
            self.__prev_sol = np.array(sol.x)
            return sol
        elif method == "anderson":
            sol = self.__solveanderson()
            self.__prev_sol = np.array(sol.x)
            return sol
        elif method == "tsls":
            sol = self.__solvetsls()
            self.__prev_sol = np.array(sol.x)
            return sol

    def __solvelm(self):
        """ Return a OptimizeResult

        Solves the system of non-linear equations in a least squares sense 
        using a modification of the Levenberg-Marquardt algorithm
        """
        return root(self.__fun,
                    self.__prev_sol,
                    jac=self.__jac,
                    method="lm",
                    options={"col_deriv": False,
                             "xtol": self.__tolerance,
                             "maxiter": self.__iterMax}
                    )

    def __solvetsls(self):
        """ Return scipy.optimize.OptimizeResult

        Solve a non-linear system using the TSLS+WD (two-step least squares) method
        """
        guess = self.__prev_sol
        for iter in range(self.__iterMax):
            jac, fun = self.__jac(guess), self.__fun(guess)
            if math.sqrt(np.matmul(fun, fun) / len(guess)) < self.__tolerance:
                return guess, iter
            dguess = np.linalg.solve(jac, fun)
            guess = guess - dguess

    def __solveanderson(self):
        return scipy.optimize.anderson(self.__fun, self.__prev_sol)

    def __jac(self, v):
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

    def __fun(self, v):
        """ Return function

        Creation of a system of nonlinear equations        
        """
        f = np.zeros([len(self.__base_dist)])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(len(list(self.__bases_coord.values())[0])):
                f[i] += math.pow(v[j]-(self.__bases_coord.get(base)[j]), 2)
            f[i] -= math.pow(self.__base_dist.get(base), 2)
        return f


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
