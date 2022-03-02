import numpy as np
from itertools import permutations
from tsp_solvers.utils import constructGraph

class DP_TSP():
    """
    Given a series of long-lat, construct a connected graph with
        these waypoints and use dynamic programming to solve TSP
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
        vertices = [i for i in range(num_v) if i != start]
        min_dist = np.inf
        next_perm = permutations(vertices)
        min_path = []
        for item in next_perm:
            curr_dist = 0
            curr = start
            for vertex in item:
                curr_dist += self.G[curr][vertex]
                curr = vertex
            curr_dist += self.G[curr][start]
            if curr_dist < min_dist:
                min_dist = curr_dist
                min_path = [start]
                min_path.extend(list(item))
                min_path.append(start)
        path = [self.vertices_dict[i] for i in min_path]
        return path, min_dist
