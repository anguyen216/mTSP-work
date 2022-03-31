# mTSP-work
keeping track of algorithm implementations for mTSP (multiple travelling salesmen)

### To-dos
- [x] Implementations of deterministic and heuristic approaches for TSP problem
- [x] Results of TSP approaches
- [x] Add sampling function to create a series of interest points that are widespread given a squared boundary of area of interest
- [x] Add requirements to run the solvers in this repo
  - [ ] Update requirements to run the solvers in this repo
- [ ] Enforce resource/distance constraint
- [ ] Implementations of approaches for mTSP problem
  - [ ] decomposition
  - [x] approximation with uniform distance constraint

# Requirements
```
python=3.10.2
matplotlib=3.5.1
numpy=1.22.3
scipy=1.7.3
mip=1.13.0
```

Google OR-tools.  Documentation of this library can be found [here](https://developers.google.com/optimization)

Installation
```
python -m pip install --upgrade --user ortools
```

# How to run the code 
Examples of how the solvers should be run are shown in `main.py`.  The examples are run using random cities coordinates (included in `cities.py`).  A custom `main` file can be written to run the solvers based on your specific need.  The current solvers will take in both `xy-coordinate` and `longitude-latitude coordinates`.  Currently, the distance between points of interest are computed using either Euclidian distance or earth surface distance formula.  If the distance between your points of interest are computed differently, please supply the distance function yourself and use it as input for the solvers.

#### For mTSP solver
The current mTSP solver uses Google OR-tools library for Python to solve the mTSP problem.  **The distance matrix and vehicle limit distance need to be an integer for the solver to work correctly.**  The solver, besides returning the route for each vehicle in coordinate format, will print out the route (in node format) and distance traveled of each vehicle to the terminal for debugging.  Follow the `main.py` file to see how to extract route in coordinate format for your own use. 

Currently, the solver output solution in `km` unit.  However, this unit can be ignored if you want to solve for solution with `meter` unit.  Just input integer meter values and ignore the printed out `km` unit in the terminal.  Just make sure that if you do this, everything should be in meter

---
# Concepts and results
### Problem statement
Given `N (N >= 1)` salesmen and `C (C >= 4)` cities that needs to be visited, find an optimal route for each saleman that meet the following requirements:
  1. each city is visited exactly once
  2. all salesmen start and finish their route at the same city
  3. the total distance of each tour is the shortest distance that meets the above requirement
Given that (1) the salesmen can visit any city from any other cities in the network; and (2) each salesman may have their own limitation in terms of resources (e.g., salesman i-th can only cover d distance).

This type of problem is considered to be NP-hard. Read more on the problem [here](https://en.wikipedia.org/wiki/Travelling_salesman_problem) and [here](https://neos-guide.org/content/multiple-traveling-salesman-problem-mtsp)

# Approaches
There are several ways to solve mTSP, but one basic solution is to build from the solution of the TSP (travelling saleman) problem, where `N = 1`.

## Approaches for TSP problem
TSP is a specific case of mTSP (or mTSP is a generic case of TSP) in which `N = 1`.  All other constraints remain.  Compares to mTSP, TSP is a smaller and simpler problem; however, it's NP complexity remains.  There are several approaches for solving TSP problem.  The two main approaches are exact solution (deterministic approach) and near-optimal solution (heuristics).
The following list the algorithms included in this repository for solving the TSP problem.

### Exact solution using Dynamic Programming (DP)
The details of the algorithm can be found [here](https://en.wikipedia.org/wiki/Held%E2%80%93Karp_algorithm#:~:text=The%20Held%E2%80%93Karp%20algorithm%2C%20also,to%20find%20a%20minimum%2Dlength)

Since this algorithm seeks to find the exact solution, its runtime is still non-polynomial.  Out of all the algorithm included in this repository, this is the most inefficient algorithm in terms of runtime.  The following graph presents the runtime of the algorithm as the sample size (number of cities) increases

<img src="./plots/dp_tsp_runtime.png" width="500">

### Exact solution using ILP (linear Integer Programming)
The details of the algorithm can be found [here](https://python-mip.readthedocs.io/en/latest/examples.html)

Again since the algorithm seeks to find the exact solution, its runtime remains non-polynomial.  However, this approach is significantly faster the DP approach and can solve instances with larger sample sizes.  The following graph presents the runtime of the algorithm as the sample size increases

<img src="./plots/ilp_tsp_runtime.png" width="500">

### Approximation solution using OPT-2 algorithm
The details of the algorithm can be found [here](https://en.wikipedia.org/wiki/2-opt)

The algorithm seeks to find near-optimal solution.  This improve the runtime significantly.  However, the algorithm's performance (in terms of finding the near-optimal and optimal solution) relies on an initial randomized route.  The algorithm can get stuck in a non-near-optimal pitfall and has no mechanism to get out of that pitfall.  To decrease the algorithm sensitivity to randomization, the `opt2` solver is run 10 times with 10 different initial randomized routes.  The following graph presents the runtime of the algorithm as the sample size increase. 

<img src="./plots/opt2_tsp_runtime.png" width="500">

Additionally, the following graph investigate the solutions output by this algorithm when compare to the exact solution (output by ILP solver)

<img src="./plots/opt2_cost_check.png" width="500">

## Approach for mTSP problem
The mTSP solver used in this repo relied on Google OR-tools for Python.  More information on this library can be found [here](https://developers.google.com/optimization)

For large set of nodes, the library can only approximate the solution.  This solver uses the `guided local search` (more info [here](https://developers.google.com/optimization/routing/routing_options#local_search_options), and [here](https://en.wikipedia.org/wiki/Guided_Local_Search)).  By default, the solver will return a solution (or report that no solution is found) within 30 seconds.  However, this can be changed by changing `time_limit` parameter of the solver.

#### Example of solver output
The block below shows an example of what the solver will print out to terminal

Note that this output is for debugging purposes.  The solver return solution in coordinate-based format along with total distance in km
```
Route for vehicle 0: 
 14 ->  0 ->  10 ->  1 ->  2 ->  8 ->  4 ->  12 ->  13 ->  3 -> 14
Distance of the route: 7500km

Route for vehicle 1: 
 14 ->  7 ->  6 ->  11 ->  9 ->  5 -> 14
Distance of the route: 8247km

Maximum of the route distance: 8247km
Total distance traveled by all vehicles: 15747 km
```

The image below visualize the above planned paths in coordinate-based format
<img src='./plots/sample_mtsp_result.png' width="500">

**Solver limitation**
- This solver will only take in distance matrix and vehicle limits in integer format.  The implementation will still take distance function and distance matrix in float format, but the distance will be automatically round up to the next integer.  Please consider this when providing distance matrix and implementing your own distance function.
- Additionally, because of the above, please only provide `v_limits` in integer format
- This solver looks for a solution that covers all given nodes (in the `waypoints` list).  If no such solution exist, it is considered to have no solution.  The solver will not drop points for you to create a solution.  Please consider this when providing `waypoints` and `v_limits`
