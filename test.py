from trilateration import Trilateration
import time
import math
import numpy as np
import matplotlib.pyplot as plt

field_x = [-5., 5.] # field dimensions x
field_y = [-5., 5.] # field dimensions y
field_z = [0., 5] # field dimensions z
count_points = 300 # amount of points
er_range = [-0.5, 0.5] # error range

bases_coord = { 1:[-2. -2, 0],
                2:[2, -2, 0],
                3:[2, 2, 0],
                4:[-2, 2, 0]
}

def get_th_points(fx: list, fy: list, fz: list) -> np.array:
    """ Return numpy.array

    Calculation of theoretical positioning points
    """
    result = np.zeros([3, count_points])
    th = np.linspace(fx[0], fx[1], count_points)
    for i, v in enumerate(th):
        result[0][i] = math.atanh(math.cos(v))
        result[1][i] = v
        result[2][i] = 2
    return result

def get_th_dist(theor: np.array, coord: dict) -> np.array:
    result = np.zeros([count_points, len(coord.keys())])
    for i, v in enumerate(result):
        pass
    return result

def dist(bases_coord: list, x: float, y: float, z: float) -> float:
    """ Return float

    Calculation of distance between points
    """
    return math.sqrt(math.pow(x-bases_coord[0], 2) \
                    + math.pow(y-bases_coord[1], 2) \
                    + math.pow(z-bases_coord[2], 2))

if __name__ == "__main__":
    th = get_th_points(field_x, field_y, field_z)

    plt.plot(th[0], th[1], ".")
    plt.show()