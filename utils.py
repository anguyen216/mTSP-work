## Includes utility functions for mTSP and TSP problems
import numpy as np
import matplotlib.pyplot as plt

def eucDist(v1, v2):
    # Compute the euclidean distance between 2 points in xy coordinates
    x1, y1 = v1
    x2, y2 = v2
    return np.sqrt( (x1 - x2)**2 + (y1 - y2)**2 )

def longLatDist(p1, p2):
    # compute the distance between 2 lat-long points in km
    # Inputs:
    #   - p1: (lat, long) of point 1
    #   - p2: (lat, long) of point 2
    # Ouput: float, distance in km
    R = 6371  # Earth's radius in km
    lat1, lon1 = p1
    lat2, lon2 = p2
    rlat1 = np.radians(lat1)
    rlat2 = np.radians(lat2)
    dlat = np.radians(lat2 - lat1)
    dlon = np.radians(lon2 - lon1)
    a = np.sin(dlat/2)**2 + np.cos(rlat1)*np.cos(rlat2) * np.sin(dlon/2)**2
    dist = 2 * R * np.arcsin(np.sqrt(a))
    return dist

def constructGraph(vertices_dict, distFunc):
    # given a dictionary of encoded vertices
    # construct a connected graph as an adjacency matrix
    # Input:
    #   - vertices_dict: dictionary of encoded vertices
    #   - distFunc: function used to compute distance between two vertices
    # Output: adjacency matrix graph G
    #   where G[i][j] = G[j][i] = dist(vertex_i, vertex_j)
    num_v = len(vertices_dict)
    G = [[0] * num_v for _ in range(num_v)]
    for i in range(num_v):
        for j in range(num_v):
            if i != j:
                p1 = vertices_dict[i]
                p2 = vertices_dict[j]
                G[i][j] = distFunc(p1, p2)
    return G

def plotPath(path):
    lons = np.array([coord[1] for coord in path])
    lats = np.array([coord[0] for coord in path])
    plt.scatter(lons, lats)
    plt.plot(lons, lats, "-")
    plt.plot(lons[0], lats[0], "ro")    # starting point: red
    plt.plot(lons[-2], lats[-2], "mo")  # penultimate point: magenta
    #plt.savefig("test.png")
    plt.show()


#p1 = [40.689202777778, -74.044219444444]
#p2 = [38.889069444444, -77.034502777778]
#print(longLatDist(p2, p1))
