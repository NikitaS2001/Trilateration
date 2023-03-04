import math
import numpy
import scipy


class Trilateration:

    """Solution of the trilateration problem
    in the form of a system of nonlinear equations
    using global optimization methods"""

    __bases_coord: dict
    __base_dist: dict
    __tolerance: float = 5e-4  # tolerance for termination
    __iterMax: int = 20  # maximum number of iterations
    # previous solution (default value [0, 0, 0])
    __prev_sol: numpy.ndarray

    def __init__(self, base_coord: dict, prev_sol: numpy.ndarray = numpy.zeros([3])) -> None:
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
            self.__prev_sol = numpy.array(sol.x)
            return sol
        elif method == "anderson":
            sol = self.__solveanderson()
            self.__prev_sol = numpy.array(sol.x)
            return sol
        elif method == "tsls":
            sol = self.__solvetsls()
            self.__prev_sol = numpy.array(sol.x)
            return sol

    def __solvelm(self):
        """ Return a OptimizeResult

        Solves the system of non-linear equations in a least squares sense 
        using a modification of the Levenberg-Marquardt algorithm
        """
        return scipy.optimize.root(self.__fun,
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
            if math.sqrt(numpy.matmul(fun, fun) / len(guess)) < self.__tolerance:
                return guess, iter
            dguess = numpy.linalg.solve(jac, fun)
            guess = guess - dguess

    def __solveanderson(self):
        return scipy.optimize.anderson(self.__fun, self.__prev_sol)

    def __jac(self, v):
        """ Return function
        bases_coord
        Creation of the Jacobian matrix
        """
        f = numpy.zeros([len(self.__base_dist.keys()), len(
            list(self.__bases_coord.values())[0])])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(len(list(self.__bases_coord.values())[0])):
                f[i][j] = 2*(v[j]-self.__bases_coord.get(base)[j])
        return f

    def __fun(self, v):
        """ Return function

        Creation of a system of nonlinear equations        
        """
        f = numpy.zeros([len(self.__base_dist)])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(len(list(self.__bases_coord.values())[0])):
                f[i] += math.pow(v[j]-self.__bases_coord.get(base)[j], 2)
            f[i] -= math.pow(self.__base_dist.get(base), 2)
        return f
