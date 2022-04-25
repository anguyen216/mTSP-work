import numpy as np
from utils import constructGraph
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class BASIC_MTSP():
    """
    Given a series of long-lat, number of vehicles, construct a
        connected graph with these waypoints and solve the mTSP
        problem given a fix depot for all vehicles
    Note that distFunc should output distance in uniform unit
    """
    def __init__(self, coordinates, distFunc):
        # coordinates - array of lat-long of waypoints of interest
        self.vertices_dict = {i: coord for i, coord in enumerate(coordinates)}
        self.coords_dict = {coord: i for i, coord in enumerate(coordinates)}
        # convert distance matrix to integer values; values are round
        #   up to the next integer
        self.data = dict()
        mat = constructGraph(self.vertices_dict, distFunc)
        self.data['distance_matrix'] = np.ceil(mat).astype(int).tolist()

    def _distanceCallback(self, from_idx, to_idx):
        """
        Returns the distance between two nodes using
            the input distance matrix
        """
        from_node = self.manager.IndexToNode(from_idx)
        to_node = self.manager.IndexToNode(to_idx)
        return self.data['distance_matrix'][from_node][to_node]

    def _demandCallback(self, from_idx):
        """
        Returns the demand of the node
        """
        from_node = self.manager.IndexToNode(from_idx)
        return self.data['demands'][from_node]

    def _getSolution(self):
        """
        Print out path and path cost for each vehicle.
        Note that distance are in uniform unit
        """
        # setup
        paths = dict()
        max_route_distance = 0
        total_cost = 0
        for vid in range(self.data['num_vehicles']):
            idx = self.routing.Start(vid)
            plan = 'Route for vehicle {}: \n'.format(vid)
            paths[vid] = []
            route_dist = 0
            while not self.routing.IsEnd(idx):
                plan += ' {} -> '.format(self.manager.IndexToNode(idx))
                paths[vid].append(self.manager.IndexToNode(idx))
                prev_idx = idx
                idx = self.solution.Value(self.routing.NextVar(idx))
                route_dist += self.routing.GetArcCostForVehicle(prev_idx, idx, vid)
            paths[vid].append(self.manager.IndexToNode(idx))
            total_cost += route_dist
            plan += '{}\n'.format(self.manager.IndexToNode(idx))
            plan += 'Distance of the route: {} \n'.format(route_dist)
            print(plan)
            max_route_distance = max(route_dist, max_route_distance)
        print('Maximum of the route distance: {}'.format(max_route_distance))
        return paths, total_cost

    def node2Coords(self, plan):
        """
        Convert the plan dictionary which includes node to path
            dictionary which include coordinate
        Input:
        - plan: the dictionary of the node-based route for each vehicle
        Output:
        - path: the dictionary of the coord-based route for each vehicle
        """
        path = dict()
        for i in range(len(plan)):
            path[i] = [self.vertices_dict[node] for node in plan[i]]
        return path

    def solve(self, start_coord, numv, vdists, demand=None, caps=None, time_limit=30):
        """
        Given starting coordinate, number of vehicle and distance limit
            of each vehicle, solve the mTSP problem. Note that the solution
            for large number of node is an approximation
        Input:
        - start_coord: tuple (lat, lon) of starting coordinate
        - numv: int, number of vehicles
        - vdists: list of float, list contains the distance limit
            of each vehicle
        - demand: list of int, indicating the demand at each location
            None as default, populate demand to be 1 for each location
        - caps: list of int, indicating the capacity of each vehicle
            None as default, populate capacity to be number of nodes in
            the graph
        - time_limit: int, indicating the time limit in seconds for the
            solver to explore solutions.  Default is 30s
        Output:
        - paths: dictionary of the coord-based route for each vehicle
        - total_cost: the total distance traveled by all vehicles
        """
        ### basic setup
        start = self.coords_dict[start_coord]
        self.data['depot'] = start
        num_nodes = len(self.coords_dict)
        self.data['num_vehicles'] = numv
        if not demand:
            self.data['demands'] = [1 for _ in range(num_nodes)]
        else:
            self.data['demands'] = demand
        if not caps:
            self.data['vehicle_capacities'] = [num_nodes for _ in range(numv)]
        else:
            self.data['vehicle_capacities'] = caps

        ### solver setup
        # create routing index manager
        self.manager = pywrapcp.RoutingIndexManager(num_nodes, numv, start)
        # create routing model
        self.routing = pywrapcp.RoutingModel(self.manager)
        transit_callback_idx = self.routing.RegisterTransitCallback(self._distanceCallback)
        # define cost of each arc
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_idx)
        demand_callback_idx = self.routing.RegisterUnaryTransitCallback(self._demandCallback)

        ### add constraints
        # add distance constraint
        dimension_name = 'Distance'
        self.routing.AddDimensionWithVehicleCapacity(
            transit_callback_idx,
            0,         # no slack
            vdists,    # vehicle maximum travel distance
            True,      # start cumul to zero
            dimension_name
        )
        distance_dimension = self.routing.GetDimensionOrDie(dimension_name)
        # TODO: figure out why the input parameter below is 100
        distance_dimension.SetGlobalSpanCostCoefficient(100)
        # add capacity constraint
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_idx,
            0,         # null capacity slack
            self.data['vehicle_capacities'],
            True,      # start cumul to zero
            'Capacity'
        )

        ### heuristic setup
        # setting first solution heuristic
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        # using guided search with time_limit of 30 seconds to get out
        # of local optimal
        search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.seconds = time_limit
        search_parameters.log_search = False

        ### get solution
        self.solution = self.routing.SolveWithParameters(search_parameters)
        if self.solution:
            plan, total_cost = self._getSolution()
            paths = self.node2Coords(plan)
            return paths, total_cost
        else:
            print('No solution found!')
