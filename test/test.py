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

bases_coord = { "0": [0, 0, 0],
                "1": [2, 0, 0],
                "2": [0, 2, 0],
                "3": [2, 2, 0.2],
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
        result[2][i] = abs(math.cos(v))
    return result

def get_th_dist(coord: dict, theor: np.array) -> np.array:
    result = list()
    for i in range(count_points):
        d = dict()
        for v in list(coord.keys()):
            d[v] = dist(coord.get(v), theor[0][i], theor[1][i], theor[2][i])
        result.append(d)
    return result

def dist(bases_coord: list, x: float, y: float, z: float = 0) -> dict:
    """ Return dict

    Calculation of distance between points
    """
    return math.sqrt(math.pow(x-bases_coord[0], 2) \
                    + math.pow(y-bases_coord[1], 2) \
                    + math.pow(z-bases_coord[2], 2))

if __name__ == "__main__":
    th = get_th_points(field_x, field_y, field_z)
    th_dist = get_th_dist(bases_coord, th)  

    cl = np.zeros([3, count_points])

    tril = Trilateration(bases_coord)

    x0 = np.zeros([len(list(bases_coord.values())[0])])

    for j in range(count_points):
        tstart = time.time()

        sol = tril.solve(th_dist[j], x0, method="lm")
        x0 = sol.x
        cl[0][j] = sol.x[0]
        cl[1][j] = sol.x[1]
        cl[2][j] = sol.x[2]

        tstop = time.time()
        print('Optimize root time', tstop-tstart)
        tstart = tstop

    fig = plt.figure()

    #plt.plot(cl[0], cl[1], "+")
    #plt.plot(th[0], th[1], ".")
    
    ax = plt.axes(projection='3d')
    ax.plot3D(th[0], th[1], th[2], ".")
    ax.plot3D(cl[0], cl[1], cl[2], "+")

    plt.show()