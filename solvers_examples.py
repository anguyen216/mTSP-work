# includes examples of how to run solvers
# Author: Anh Nguyen
import numpy as np
import matplotlib.pyplot as plt
from utils import longLatDistKm, plotPath, plotMultiplePaths
from tsp_solvers.dp_solver import DP_TSP
from tsp_solvers.ILP_solver import ILP_TSP
from tsp_solvers.opt2_solver import OPT2_TSP
from cities import WAYPOINTS, NAMES_DICT
from mtsp_solvers.basic_solver import BASIC_MTSP
import time

def main():
    # SETUP
    N = len(WAYPOINTS)
    opt2_runtime = []
    opt2_costs = []
    opt2_sizes = [i for i in range(3, N, 3)]
    ilp_costs = []
    # dp_runtime = []
    # ilp_runtime = []
    ilp_sizes = [i for i in range(3, 26, 3)]
    # dp_sizes = [i for i in range(3, 13, 3)]

    # testing basic mTSP solver
    indices = np.random.choice(40, size=15, replace=False)
    waypoints = [WAYPOINTS[idx] for idx in indices]
    start = np.random.randint(0, len(waypoints))
    bmtsp = BASIC_MTSP(waypoints, longLatDistKm)
    num_v = 2
    colors = ['steelblue', 'orange']
    v_limits = [100000] * num_v
    sol = bmtsp.solve(waypoints[start], num_v, v_limits)
    if sol:
        paths, cost = sol
        print("Total distance traveled by all vehicles:", cost, "km")
        pname = "./plots/sample_mtsp_result.png"
        plotMultiplePaths(paths, colors, save_plot=True, plot_name=pname)


    # Check OPT-2 near-optimal solution
    # for s in ilp_sizes:
    #     print(s)
    #     indices = np.random.randint(0, 26, size=s)
    #     waypoints = [WAYPOINTS[idx] for idx in indices]
    #     start = np.random.randint(0, s)
    #     ilp = ILP_TSP(waypoints, longLatDistKm)
    #     opt2 = OPT2_TSP(waypoints, longLatDistKm)
    #     ilp_path, ilp_cost =  ilp.solve(waypoints[start])
    #     opt2_path, opt2_cost = opt2.solve(waypoints[start])
    #     opt2_costs.append(opt2_cost)
    #     ilp_costs.append(ilp_cost)
    # plt.plot(ilp_sizes, ilp_costs, label='ILP cost')
    # plt.plot(ilp_sizes, opt2_costs, label='OPT-2 cost')
    # plt.xlabel('number of waypoints')
    # plt.ylabel('solution cost in km')
    # plt.legend()
    # plt.title('Check OPT-2 near-optimal solution')
    # plt.savefig("plots/opt2_cost_check.png")

    # Plot OPT-2 runtime
    # for s in range(3, N, 3):
    #     print(s)
    #     indices = np.random.randint(0, N, size=s)
    #     waypoints = [WAYPOINTS[idx] for idx in indices]
    #     start = np.random.randint(0, s)
    #     opt2 = OPT2_TSP(waypoints, longLatDistKm)
    #     start_time = time.time()
    #     opt2_path, opt2_cost = opt2.solve(waypoints[start])
    #     opt2_runtime.append(time.time() - start_time)
    #     opt2_costs.append(opt2_cost)
    # plt.plot(opt2_sizes, opt2_runtime)
    # plt.xlabel('number of waypoints')
    # plt.ylabel('runtime in seconds')
    # plt.title('runtime of OPT-2 approximation for TSP problem')
    # plt.savefig("plots/opt2_tsp_runtime.png")

    # Plot ILP runtime
    # for s in ilp_sizes:
    #     # randomize the waypoints and number of waypoints
    #     # pick a random starting point
    #     print(s)
    #     indices = np.random.randint(0, 26, size=s)
    #     waypoints = [WAYPOINTS[idx] for idx in indices]
    #     start = np.random.randint(0, len(waypoints))
    #     # setup solvers
    #     ilp = ILP_TSP(waypoints, longLatDistKm)
    #     # time solvers
    #     start_time = time.time()
    #     ilp_path, ilp_cost =  ilp.solve(waypoints[start])
    #     ilp_runtime.append(time.time() - start_time)
    # # ILP plot
    # plt.plot(ilp_sizes, ilp_runtime)
    # plt.xlabel('number of waypoints')
    # plt.ylabel('runtime in seconds')
    # plt.title('runtime of ILP solver for TSP problem')
    # plt.savefig("plots/ilp_tsp_runtime.png")

    # Plot DP runtime
    # for s in dp_sizes:
    #     print(s)
    #     indices = np.random.randint(0, 13, size=s)
    #     waypoints = [WAYPOINTS[idx] for idx in indices]
    #     start = np.random.randint(0, len(waypoints))
    #     dp = DP_TSP(waypoints, longLatDistKm)
    #     start_time =  time.time()
    #     dp_path, dp_cost = dp.solve(waypoints[start])
    #     dp_runtime.append(time.time() - start_time)
    #
    # # DP plot
    # plt.plot(dp_sizes, dp_runtime)
    # plt.xlabel('number of waypoints')
    # plt.ylabel('runtime in seconds')
    # plt.title('runtime of DP solver for TSP problem')
    # plt.savefig("plots/dp_tsp_runtime.png")


if __name__ == "__main__":
    main()
