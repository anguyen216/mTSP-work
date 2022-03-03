# The opt2 solver and cost function are adapted from this post
# source: https://stackoverflow.com/questions/53275314/2-opt-algorithm-to-solve-the-travelling-salesman-problem-in-python
# more information about opt-2 algorithm can be found here
# https://en.wikipedia.org/wiki/2-opt
import numpy as np
import random
from tsp_solvers.utils import constructGraph, eucDist

class OPT2_TSP():
    """
    Given a series of long-lat, construct a connected graph with
        these waypoints and use Opt-2 approximation algorithm (heuristic)
        to solve for near-optimal solution given a starting point.
        Start with a randomized route and iteratively improve on that route
        by removing path crossing in each neighborhood of waypoints.
    """
    def __init__(self, coordinates, distFunc):
        # coordinates - array of lat-long of waypoints of interest
        # self.G - encoded cost map among vertices
        self.vertices_dict = {i: coord for i, coord in enumerate(coordinates)}
        self.coords_dict = {coord: i for i, coord in enumerate(coordinates)}
        self.G = constructGraph(self.vertices_dict, distFunc)

    def cost(self, route):
        # compute the cost of the given route
        G = np.array(self.G)
        r = np.array(route)
        return G[np.roll(r, 1), r].sum()

    def opt2(self, route):
        # iteratively improving the current route using
        # opt-2 approximation algorithm
        best = route
        improved = True
        while improved:
            improved = False
            for i in range(1, len(route) - 2):
                for j in range(i+1, len(route)):
                    if j - i == 1: continue
                    new_route = route[:]
                    new_route[i:j] = route[j-1 : i-1: -1]
                    if self.cost(new_route) < self.cost(route):
                        best = new_route
                        improved = True
            route = best
        return best, self.cost(best)

    def solve(self, start_coord, iter=10):
        # run opt2 10 times with different randomization of initial
        # route to decrease the algorithm sensitivity to initial route
        start = self.coords_dict[start_coord]
        num_v = len(self.G)
        vertices = [i for i in range(num_v) if i != start]
        opt_route, best_cost = [], float("inf")
        for _ in range(iter):
            init_route = [start]
            tmp = random.sample(vertices, len(vertices))
            init_route.extend(tmp)
            init_route.append(start)
            curr_route, curr_cost = self.opt2(init_route)
            if curr_cost < best_cost:
                best_cost = curr_cost
                opt_route = curr_route
        path = [self.vertices_dict[i] for i in opt_route]
        return path, best_cost
