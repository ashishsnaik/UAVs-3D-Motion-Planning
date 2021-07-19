import os
import time
import pickle
import numpy as np
import numpy.linalg as LA
import networkx as nx
import matplotlib.pyplot as plt
from queue import PriorityQueue
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString

class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]
    
    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)

def extract_polygons(map_data, safety_distance=5):
    # NOTE: Polygon includes obstacle + the required safety distance around the obstacle

    polygons = []
    for i in range(map_data.shape[0]):
        # obstacle centers and width/depth on each side
        x_north, y_east, alt, d_north, d_east, d_alt = map_data[i, :]
        delta_x_north_and_safety = d_north + safety_distance
        delta_y_east_and_safety = d_east + safety_distance
        d_alt_safety = d_alt + safety_distance
              
        # obstacle corners in terms of cartesian-coordinates
        # horizontal limits
        x1 = x_north - delta_x_north_and_safety
        x2 = x_north + delta_x_north_and_safety
        # verticle limits
        y1 = y_east - delta_y_east_and_safety
        y2 = y_east + delta_y_east_and_safety

        # polygon corners (close the polygon with the 5th coordinate as the first one)
        corners = [(x1, y1), (x1, y2), (x2, y2), (x2, y1), (x1, y1)]
        
        # Compute the height of the polygon
        height = alt + d_alt_safety

        p = Poly(corners, height)
        polygons.append(p)

    return polygons


# Route Planner 
# Searches route path 
# Works with NED coordinates (Local Position) only!
class RoutePlanner():

    def __init__(self, target_altitude=50, safety_distance=5, graph_num_samples=3000, graph_k=30):
        
        # Read in obstacle map which contains obstacle center in X-pos, Y-pos, Z-pos (Altitude) and 
        # delta_x, delta_y, delta_z (x-half-width, y-half-width, z-half-height from center
        map_data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # drone target altitude to fly at
        self._target_altitude = target_altitude
        # safety distance from obstacles
        self._safety_distance = safety_distance
        # number of sample points to use for graph creation
        self._num_samples = graph_num_samples
        # number of neighnoring nodes to connect for graph creation
        self._route_graph_k = graph_k
        
        # route graph name and file path to store and retrieve
        alt_str = str(self._target_altitude) if self._target_altitude is not None else "rand"
        samp_k_safety_str = "samp" + str(self._num_samples) + "_k" + str(self._route_graph_k) + "_safety" + str(self._safety_distance)
        self._route_graph_name = "route_graph_alt" + alt_str + "_" + samp_k_safety_str
        self._route_graph_file_name = self._route_graph_name + ".gpickle"
        self._route_graph_file_path = os.path.join("graph", self._route_graph_file_name)
        
        # map obstacles
        self._polygons = extract_polygons(map_data, safety_distance=self._safety_distance)

        self._xmin = np.min(map_data[:, 0] - map_data[:, 3])
        self._xmax = np.max(map_data[:, 0] + map_data[:, 3])
        self._ymin = np.min(map_data[:, 1] - map_data[:, 4])
        self._ymax = np.max(map_data[:, 1] + map_data[:, 4])
        self._zmin = 0
        # limit z-axis to max obstacle height 
        self._zmax = np.max(map_data[:, 1] + map_data[:, 4])
        
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to 
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((map_data[:, 3], map_data[:, 4]))
        
        # create the obstacles tree for efficient search
        print("Creating obstacle tree...")
        centers = np.array([p.center for p in self._polygons])
        self._obstacles_tree = KDTree(centers, metric='euclidean')
        
        #
        # load or create the route graph
        #
        
        if os.path.exists(self._route_graph_file_path):
            print("Loading Graph ", self._route_graph_file_path)
            self._route_graph = nx.read_gpickle(self._route_graph_file_path)
            print("...Done")
        else:
            
            print("Creating Graph...")

            # sample random points in 3d-free space in the map
            print("Getting sample nodes...")
            self._sample_nodes = self.__sample()
            print("Num sample nodes: ", len(self._sample_nodes))
            print("Creating graph using {0} neighbouring nodes...".format(str(self._route_graph_k)))
            t0 = time.time()
            self._route_graph = self.__create_graph(self._sample_nodes)
            print("Graph creation DONE in {0} seconds!".format(time.time()-t0))
            print("Saving graph...")
            nx.write_gpickle(self._route_graph, self._route_graph_file_path)
            print("Graph saved to ", self._route_graph_file_path)

        self._route_graph_nodes = list(self._route_graph.nodes)
#         print("Graph Nodes: ", self._route_graph_nodes)
#         print(list(self._route_graph.nodes(data=True)))

        # create a tree of the graph nodes so we can search for 
        # the closest graph-node to any map location
        self._route_graph_tree = KDTree(self._route_graph_nodes)


    @property
    def polygons(self):
        return self._polygons

    # heuristic cost function
    def __heuristic(self, n1, n2):
        return LA.norm(np.array(n2)-np.array(n1))

    # if optimize=True, checks only those polygons that are in the certain radius 
    # radius = hypotenuse of half-distance and max obstacle width plus safety distance
    # else checks all polygons
    def __can_connect(self, n1, n2, optimize=True):
        
        if optimize is True:
            # get the radius to scan for obstacles around both points
            half_dist = LA.norm(np.array(n2[0:2])-np.array(n1[0:2])) / 2.0
            max_obstacle_length = self._max_poly_xy + self._safety_distance
            radius = np.sqrt(half_dist**2 + max_obstacle_length**2)
            # get indices of obstacles from both the points
            idxs1 = list(self._obstacles_tree.query_radius(np.array(n1[0:2]).reshape(1, -1), r=radius))[0]        
            idxs2 = list(self._obstacles_tree.query_radius(np.array(n2[0:2]).reshape(1, -1), r=radius))[0]        
            # get the set of obstacles to scan for collision
            idxs = list(set(idxs1.tolist()+idxs2.tolist()))
            
            polygons = np.array(self._polygons)[idxs]
        else:
            polygons = self._polygons
            
        # check for collision
        line = LineString([n1, n2])
        for p in polygons:
            if p.crosses(line) and p.height > min(n1[2], n2[2]):
                return False
        
        return True

    def __sample(self):
        """Implemented with a k-d tree for efficiency."""
        # xvals are "east" vals and yvals are "north" vals on the map
        xvals = np.random.uniform(self._xmin, self._xmax, self._num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, self._num_samples)
        if self._target_altitude is None:
            zvals = np.random.uniform(self._zmin, self._zmax, self._num_samples)
        else:
            zvals = np.full(self._num_samples, self._target_altitude, dtype=float)
        
        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            in_collision = False
            idxs = list(self._obstacles_tree.query_radius(
                np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            
            if len(idxs) > 0:
                for ind in idxs: 
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True

            if not in_collision:
                pts.append(s)
                
        return pts

    def __create_graph(self, nodes):
        g = nx.Graph()
        print("Creating graph...")
        temp_num_node = len(nodes)
        node_tree = KDTree(nodes)
        for i, n1 in enumerate(nodes):
            print("Processing node {0} of {1}: {2}".format(i, temp_num_node, n1))
            distances, indices = node_tree.query([n1], self._route_graph_k, return_distance=True)
            for dist, idx in zip(distances[0], indices[0]):
                n2 = nodes[idx]
                if n2 == n1:
                    continue
                if self.__can_connect(n1, n2):
                    # connect the graph edge with weight as the distance between the nodes
                    g.add_edge(n1, n2, weight=dist)
        print("...Graph creation DONE!")
        return g

    # gets the nearest neighbor node to a route graph node; either the node index or the node itself
    def __get_nearest_neighbor_node(self, ned_loc, idx_only=False, ned_loc_is_graph_node=True):
        retVal = None
        # if ned_loc (for which to search the NN node is a graph node, use idx 1 as idx 0 will the ned_loc itself)
        node_idx_to_use = 1 if ned_loc_is_graph_node is True else 0
        nn_node_indices = self._route_graph_tree.query([ned_loc], k=2, return_distance=False)
#         print("node_indices: ", nn_node_indices)
        if idx_only is False:
            retVal = self._route_graph_nodes[nn_node_indices[0][node_idx_to_use]]
        else:
            retVal = nn_node_indices[0][node_idx_to_use]
        
        return retVal

    # searches for the closest connectable graph node to a map NED location; 
    # returns: tuple
    # - if map_ned loc is valid: closest graph node (or node idx), True/False (whether connectable or not)
    # - otherwise: None, Dont_Care
    def __get_closest_connectable_graph_node(self, map_ned, k=5, idx_only=False): 
        is_connectable = False
        node_idx = 0
        closest_node = None
        
        # first check whether the map location is in free space (isn't inside an obstacle/bulding)
        if self.__is_location_in_free_space(map_ned):
        
            # get 5 closest graph nodes, and return the closest one that has direct arial connectivity 
            # if one is present, else return none.
            closest_node_indices = self._route_graph_tree.query([map_ned], k=k, return_distance=False)[0]
#             print("closest_node_indices: ", closest_node_indices)
            
            if idx_only is False:
                closest_node = self._route_graph_nodes[closest_node_indices[node_idx]]
            else:
                closest_node = closest_node_indices[node_idx]
                
            for idx in closest_node_indices:
                current_node = self._route_graph_nodes[idx]
                is_connectable = self.__can_connect(map_ned, current_node)
#                 print(map_ned, current_node, is_connectable)
                if is_connectable:
                    node_idx = idx
                    closest_node = self._route_graph_nodes[node_idx] if idx_only is False else node_idx
                    break

            if is_connectable is False:
                print("Warning! Graph Node NOT Connectable To Location ", map_ned)
        
        else:
            print("Warning! Input Map Location (NED) Not In Free Space")
        
        return closest_node, is_connectable

    def __is_location_in_free_space(self, ned_loc, k=5):
        in_free_space = True
        idxs = list(self._obstacles_tree.query(np.array([ned_loc[0], ned_loc[1]]).reshape(1, -1), 
                                               k=k, return_distance=False)[0])
#         print(idxs)

        if len(idxs) > 0:
            for idx in idxs: 
                p = self._polygons[int(idx)]
                if p.contains(ned_loc) and p.height >= ned_loc[2]:
                    in_free_space = False
                    break

        return in_free_space
        
    # searches and returns a graph path if found, empty list otherwise.
    def __search_path(self, start_node, goal_node):

        """Modified A* to work with NetworkX graphs."""

        path = []
        queue = PriorityQueue()
        queue.put((0, start_node))
        visited = set(start_node)

        branch = {}
        found = False
        
        while not queue.empty():
            item = queue.get()
            current_cost = item[0]
            current_node = item[1]

            if current_node == goal_node:        
                found = True
                break
            else:
                for next_node in self._route_graph[current_node]:
                    cost = self._route_graph.edges[current_node, next_node]['weight']
                    new_cost = current_cost + cost + self.__heuristic(next_node, goal_node)

                    if next_node not in visited:                
                        visited.add(next_node)               
                        queue.put((new_cost, next_node))

                        branch[next_node] = (new_cost, current_node)

        path = []
        path_cost = 0
        if found:
            # retrace steps
            path = []
            n = goal_node
            path_cost = branch[n][0]
            while branch[n][1] != start_node:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
        else:
            print("Path Not Found")

        return path[::-1], path_cost

    # prunes graph path and sets the drone orientation using wp1 and wp2
    def __prune_path_and_orient(self, waypoints):
        print("Pruning Waypoints...")
        pruned_waypoints = []
        num_wp = len(waypoints)
        if num_wp > 3:
            s_idx = 0
            e_idx = 0
            wp1 = waypoints[s_idx]
            pruned_waypoints.append([int(round(wp1[0])), int(round(wp1[1])), int(round(wp1[2])), int(round(wp1[3]))])
            while e_idx < num_wp-1:
                print(waypoints[s_idx][0:3], " - ", waypoints[e_idx][0:3])
                if self.__can_connect(waypoints[s_idx][0:3], waypoints[e_idx][0:3]) is True:
                    print("Can Connect")
                    e_idx +=1
                else:
                    print("Can NOT Connect")
                    s_idx = e_idx-1
                    wp2 = waypoints[s_idx]
                    orientation = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
                    pruned_waypoints.append([int(round(wp2[0])), int(round(wp2[1])), int(round(wp2[2])), orientation])
                    wp1 = wp2

            # add the last two waypoints
            wp2 = waypoints[e_idx-1]
            orientation = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
            pruned_waypoints.append([int(round(wp2[0])), int(round(wp2[1])), int(round(wp2[2])), orientation])

            wp1 = wp2
            wp2 = waypoints[e_idx]
            orientation = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
            pruned_waypoints.append([int(round(wp2[0])), int(round(wp2[1])), int(round(wp2[2])), orientation])
        else:
            print("Waypoints: Nothing to prune!")
            return waypoints

        return pruned_waypoints
    
    def get_flight_waypoints(self, start_ned, goal_ned, prune_path=True):
        
        # NOTE: The complete route path will be:
        # Start_Location (map) ==> Graph_Start_Node ==> Graph_Goal_Node ==> Goal_Location (map)
        # Constraints:
        # Map_Start_Location ==> Graph_Start_Node MUST BE FOUND
        # Graph_Start_Node ==> Graph_Goal_Node MUST BE FOUND
        # IF Graph_Goal_Node ==> Map_Goal_Location is not found, then we stop at the Graph_Goal_Node. 
        
        # waypoints to return
        waypoints = []
        # use graph node as destination because goal location is not feasible?
        use_graph_node_as_dstn = False
        
        # get the graph node that is closest to the start location 
        # if the below function returns None, it means the specified start 
        # is not in free (open) space but inside a building/obstacle
        graph_start_node, connectable = self.__get_closest_connectable_graph_node(start_ned)
        
        if graph_start_node is not None and connectable is True:
        
            # get the graph node that is closest to the goal location
            graph_goal_node, connectable = self.__get_closest_connectable_graph_node(goal_ned)
            
            # if the goal location is inside an obstable, make the goal location as the graph goal node
            if graph_goal_node is None:
                print("Warning! Goal location not in Free (Open) Space")
                print("Using Closest Graph Node As Destination")
                graph_goal_node = self.__get_nearest_neighbor_node(goal_ned, ned_loc_is_graph_node=False)
                use_graph_node_as_dstn = True
                
            elif connectable is False:
                print("Warning! Goal location Un-Connectable to Route Graph")
                print("Using Closest Graph Node As Destination")
                use_graph_node_as_dstn = True

#             print("Route Planner: start_ned: ", start_ned)
#             print("Route Planner: graph_start_node: ", graph_start_node)
#             print("Route Planner: graph_goal_node: ", graph_goal_node)
#             print("Route Planner: goal_ned: ", goal_ned)

            # first we check whether there is a graph path connecting the start and goal nodes
            graph_path, cost = self.__search_path(graph_start_node, graph_goal_node)
            
            print("Route Planner - Found Path ({0} Waypoints): {1}".format(len(graph_path), graph_path))

            # if we have a graph path, find start_location - to - start_node and goal_node - to - goal_location connections
            if len(graph_path) > 0:
                # Start_Location (map) ==> Graph_Start_Node
                # add additional waypoint that is at start NE and TARGET_ALTITUDE, before the actual start NED
                waypoints.append([int(round(start_ned[0])), int(round(start_ned[1])), self._target_altitude, 0])
                
                # Start_Location (map) ==> Graph_Start_Node ==> Graph_Goal_Node
                for p in graph_path:
                    waypoints.append([int(round(p[0])), int(round(p[1])), int(np.ceil(p[2])), 0])
                
                # Graph_Goal_Node ==> Goal_Location (map)
                if use_graph_node_as_dstn is False:
                    # add additional waypoint that is at goal NE and TARGET_ALTITUDE, before the actual goal NED
                    waypoints.append([int(round(goal_ned[0])), int(round(goal_ned[1])), self._target_altitude, 0])
                    waypoints.append([int(round(goal_ned[0])), int(round(goal_ned[1])), int(-np.ceil(goal_ned[2])), 0])
                
                # else add destination as last graph path node and zero altitude
                else:
                    p = graph_path[len(graph_path)-1]
                    waypoints.append([int(round(p[0])), int(round(p[1])), 0, 0])
                    
            else:
                print("Warning! No Graph Node Path Found...")
        else:
            print("Error! Start location not in Free (Open) Space OR Un-Connectable to Route Graph")
            print("Please Change Start Location!")
            
        
        print("All Un-Pruned Waypoints: {0}".format(waypoints))
        
        if prune_path is True and len(waypoints) > 0:
            pruned_waypoints = self.__prune_path_and_orient(waypoints)
            return pruned_waypoints
        else:
            return waypoints

    # plots the route graph at target altitude
    def plot_route_graph(self):

        plt.rcParams['figure.figsize'] = 14, 14

        # visualize the route graph at the target altiture!
        fig = plt.figure()

        # plot the polygons
        for poly1 in self._polygons:
            if poly1.height >= self._target_altitude:
                x, y = poly1._polygon.exterior.xy
                plt.plot(y, x)

        # # Draw all nodes connected or not in blue
        # for n1 in self._route_planner._sample_nodes:
        #    plt.scatter(n1[1], n1[0], c='gray')

        # Draw connected nodes
        for n1 in self._route_graph_nodes:
            plt.scatter(n1[1], n1[0], c='yellow')

        # Draw edges (color: light blue)
        for (n1, n2) in self._route_graph.edges:    
            plt.plot([n1[1], n2[1]], [n1[0], n2[0]], c='#00FFFF' , alpha=0.5)

        plt.xlabel('East')
        plt.ylabel('North')

        plt.show()
