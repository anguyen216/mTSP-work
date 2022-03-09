# the ILP solver is writen by Sandipan Dey.
# source code for solver can be found here: https://sandipanweb.wordpress.com/2020/12/08/travelling-salesman-problem-tsp-with-python/
# this solver is adapted from the above source code to solve for TSP tour
#   given user-input starting point
from mip import Model, xsum, minimize, BINARY
from itertools import product
from utils import constructGraph

class ILP_TSP():
    """
    Given a series of long-lat, construct a connected graph with
        these waypoints and use integer linear programming to solve TSP
        problem given a starting point
    """
    def __init__(self, coordinates, distFunc):
        # coordinates - array of lat-long of waypoints of interest
        self.vertices_dict = {i: coord for i, coord in enumerate(coordinates)}
        self.coords_dict = {coord: i for i, coord in enumerate(coordinates)}
        self.G = constructGraph(self.vertices_dict, distFunc)

    def solve(self, start_coord):
        start = self.coords_dict[start_coord]
        num_v = len(self.G)
        V = set(range(num_v))
        model = Model()
        x = [[model.add_var(var_type=BINARY) for j in V] for i in V]
        y = [model.add_var() for i in V]
        model.objective = minimize(xsum(self.G[i][j] * x[i][j] for i in V for j in V))

        for i in V:
            model += xsum(x[i][j] for j in V - {i}) == 1
        for i in V:
            model += xsum(x[j][i] for j in V - {i}) == 1
        for (i, j) in product(V - {start}, V - {start}):
            if i != j:
                model += y[i] - (num_v + 1)*x[i][j] >= y[j] - num_v
        model.optimize()
        if model.num_solutions:
            nc = start
            cycle = [nc]
            while True:
                nc = [i for i in V if x[nc][i].x >= 0.99][0]
                cycle.append(nc)
                if nc == start:
                    break
        path = [self.vertices_dict[i] for i in cycle]
        return path, model.objective_value
