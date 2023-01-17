import math
import numpy
import scipy

class Trilateration:

    """Solution of the trilateration problem
    in the form of a system of nonlinear equations
    using global optimization methods"""

    __bases_coord: dict
    __base_dist: dict
    __tolerance: float = 5e-4 # tolerance for termination
    __iterMax: int = 20 # maximum number of iterations

    def __init__(self, base_coord: dict) -> None:
        self.__bases_coord = base_coord
    
    def set_bases_coord(self, base_coord: dict) -> None:
        self.__bases_coord = base_coord

    def get_bases_coord(self) -> dict:
        return self.__bases_coord

    def set_iterMax(self, iterMax: int) -> None:
        self.__iterMax = iterMax

    def get_iterMax(self) -> int:
        return self.__iterMax

    def solve(self, base_dist :dict, guess :numpy.array, method="lm") -> scipy.optimize.OptimizeResult:

        self.__base_dist = base_dist

        if method == "lm":
            return self.__solvelm(guess)
        elif method == "anderson":
            return self.__solveanderson(guess)
        elif method == "tsls":
            return self.__solvetsls(guess)

    def __solvelm(self, guess: numpy.array) -> scipy.optimize.OptimizeResult:
        """ Return a OptimizeResult

        Solves the system of non-linear equations in a least squares sense 
        using a modification of the Levenberg-Marquardt algorithm
        """
        return scipy.optimize.root(self.__fun,
                                    guess,
                                    jac=self.__jac,
                                    method="lm",
                                    options={"col_deriv" :False,
                                             "xtol" :self.__tolerance,
                                             "maxiter" :self.__iterMax}
                                    
                                    )

    def __solvetsls(self, guess: numpy.array) -> scipy.optimize.OptimizeResult:
        """ Return scipy.optimize.OptimizeResult
        
        Solve a non-linear system using the TSLS+WD (two-step least squares) method
        """
        
        for iter in range(self.__iterMax):
            jac, fun = self.__jac(guess), self.__fun(guess)
            if math.sqrt(numpy.matmul(fun, fun) / len(guess)) < self.__tolerance:
                return guess, iter
            dguess = numpy.linalg.solve(jac, fun)
            guess = guess - dguess

    def __solveanderson(self, guess: numpy.array) -> scipy.optimize.OptimizeResult:
        return scipy.optimize.anderson(self.__fun, guess)

    def __jac(self, v):
        """ Return function

        Creation of the Jacobian matrix
        """
        f = numpy.zeros([len(self.__base_dist.keys()), len(list(self.__bases_coord.values())[0])])
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(len(list(self.__bases_coord.values())[0])):
                f[i][j] = 2*(v[j]-self.__bases_coord.get(base)[j])
        return f

    def __fun(self, v):
        """ Return function

        Creation of a system of nonlinear equations        
        """
        # f = numpy.zeros([len(self.__base_dist)])
        f = numpy.zeros(7)
        for i, base in enumerate(list(self.__base_dist.keys())):
            for j in range(len(list(self.__bases_coord.values())[0])):
                f[i] += math.pow(v[j]-self.__bases_coord.get(base)[j], 2)
            f[i] -= math.pow(self.__base_dist.get(base), 2)
        return f