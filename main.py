# sampling simulation and solver results for mTSP problem
# Author: Anh Nguyen
import numpy as np
import matplotlib.pyplot as plt
from utils import longLatDistKm, longLatDistM, plotPath, plotMultiplePaths
from utils import samplingPointSim, sobolSamples
from mtsp_solvers.basic_solver import BASIC_MTSP

def main():
    ### Boundary coordinates of ottway site (lat, lon)
    #   bottom left: 34.7844656, -76.5719868
    #   top right: 34.7840274, -76.5690488
    bot_left = (34.784027, -76.571366)
    top_right = (34.786204, -76.569048)

    ### Setups
    # set number of samples for the whole site
    # divide the rectangle site into (nrows - 1) * (ncols - 1) grid
    # create a simulated set of points for sampling
    # find the sampling points for the boat
    # the rest of the points will be sampled by the other vehicles
    # define boat and other vehicles contraints
    #   total distance constraint (in meters)
    #   number of vehicles constraint
    #   depot
    num_samples = 20
    nrows, ncols = 5, 3
    boat_dist = 3000  # 3000 meters
    drone_dist = 3000
    ndrones = 3
    single_drone_cap = 5
    seed = 66  # to create reproduceable result, can be randomized
    sim = samplingPointSim(bot_left, top_right, num_samples, longLatDistM, seed)
    samples = sim.samples.tolist()
    samples = [tuple(p) for p in samples]
    boat_points = sim.nearest2Centroids(nrows, ncols).tolist()
    boat_points = [tuple(p) for p in boat_points]
    drone_points = [tuple(p) for p in samples if p not in boat_points]
    boat_caps = [len(boat_points)]
    drone_caps = [single_drone_cap for _ in range(ndrones)]
    drone_dists = [drone_dist] * ndrones
    depot = boat_points[0]
    drone_points.append(depot)

    # creating solvers and getting solution paths
    boat_solver = BASIC_MTSP(boat_points, longLatDistM)
    drone_solver = BASIC_MTSP(drone_points, longLatDistM)
    boat_sol = boat_solver.solve(depot, 1, [boat_dist], caps=boat_caps)
    drone_sol = drone_solver.solve(depot, ndrones, drone_dists, caps=drone_caps)

    # plotting results
    if boat_sol:
        boat_path, _ = boat_sol
    if drone_sol:
        drone_paths, _ = drone_sol
    drone_paths[ndrones] = boat_path[0]
    colors = ["steelblue"] * ndrones
    colors.append("orange")
    pname = "./plots/simulated_samples_sol_cap_constraint.png"
    plotMultiplePaths(drone_paths, colors, save_plot=True, plot_name=pname)


if __name__ == "__main__":
    main()
