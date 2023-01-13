import math
import numpy
import scipy

class Trilateration:

    """Solution of the trilateration problem
    in the form of a system of nonlinear equations
    using global optimization methods"""

    def __init__(self, base_coord :dict) -> None:
        self.__bases_coord = base_coord
        self.__visible_bases = set(base_coord.keys())
    
    def solve(self, base_dist:dict, guess :numpy.array) -> numpy.array:
        pass

    def __jac(self, x)-> function:
        pass

    def __fun(self, x) -> function:
        pass

