import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global

from route_planner import RoutePlanner

import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = 14, 14

np.set_printoptions(suppress=True)

# global constants
TARGET_ALTITUDE = 50
SAFETY_DISTANCE = 5
ROUTE_GRAPH_NUM_SAMPLES = 3000
ROUTE_GRAPH_K = 30

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class GlobalLocation():
    def __init__(self, lon=0.0, lat=0.0, alt=0.0):
        self._lon = float(lon)
        self._lat = float(lat)
        self._alt = float(alt)
    
    @property
    def location(self):
        return (self._lon, self._lat, self._alt)
    
    @property
    def lat(self):
        return self._lat
    
    @property
    def lon(self):
        return self._lon
    
    @property
    def alt(self):
        return self._alt
        
    def get_location(self):
        return (self._lon, self._lat, self._alt)
    
    def set_location(self, lon, lat, alt):
        self._lon = lon
        self._lat = lat
        self._alt = alt

        
class MotionPlanning(Drone):

    def __init__(self, connection=None, threaded=False, target_altitude=TARGET_ALTITUDE, 
                 safety_distance=SAFETY_DISTANCE, graph_num_samples=ROUTE_GRAPH_NUM_SAMPLES, 
                 graph_k=ROUTE_GRAPH_K, use_grid=False, prune_path=True):
        
#         super().__init__(connection)
        
        # after connection init (super().__init__) the graph building runs very slow
        # so even before we init the connection, first initialize the route planner

        self._threaded = threaded
        self._target_altitude = target_altitude
        self._use_grid = use_grid
        self._prune_path = prune_path
        
        # for MavLink (or any other) connection
        self._connection = connection
        self._connection_initialized = False
        
        self._in_mission = False
        self._end_mission = False
        
        # get the map center lon/lat
        map_center_lon = 0.0
        map_center_lat = 0.0
        with open('colliders.csv', 'r') as f:
            csv_reader = csv.reader(f);
            first_line = next(csv_reader)
            map_center_lon = float(first_line[1].split()[1])
            map_center_lat = float(first_line[0].split()[1])
        
        # Read in obstacle map
        self._data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # find the min and max longitude and latitude
        # here x in north (latitude) and y is east (longitude)
        # x (north) here should be y (up) in Cartesian-Coords and 
        # y (east) here is y (right) in Cartesian-Coords.  
        x_north_min = np.min(self._data[:, 0] - self._data[:, 3])
        x_north_max = np.max(self._data[:, 0] + self._data[:, 3])
        y_east_min = np.min(self._data[:, 1] - self._data[:, 4])
        y_east_max = np.max(self._data[:, 1] + self._data[:, 4])
        
        # the bottom-left corner and top-right corner will give is the min and max lon-lat values.
        # local_to_global(...) takes in NED sequence and returns lon-lat-alt sequence
        min_lla = local_to_global([x_north_min, y_east_min, 0.0], [map_center_lon, map_center_lat, 0.0])
        max_lla = local_to_global([x_north_max, y_east_max, 0.0], [map_center_lon, map_center_lat, 0.0])
        
        self._longitude_min, self._latitude_min, _ = min_lla
        self._longitude_max, self._latitude_max, _ = max_lla

        
        if not self._use_grid:
            self._route_planner = RoutePlanner(target_altitude = self._target_altitude,
                                               safety_distance=safety_distance,
                                               graph_num_samples=graph_num_samples,
                                               graph_k=graph_k)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.check_state = {}
        
        # global home, start, and goal locations in (lon/lat/alt)
        self.global_home_loc = GlobalLocation(map_center_lon, map_center_lat, 0.0)
        self.global_start_loc = None
        self.global_goal_loc = None
        
        print("Global Home (lat/lon/alt): ", self.global_home_loc.location)

        # initial state
        self.flight_state = States.MANUAL
        
    def __del__(self):
#         self.stop()
        self._end_mission = True
        self.__end_mission()

    def plot_route_graph(self):
        if not self._use_grid:
            self._route_planner.plot_route_graph()
        else:
            print("Using grid; no graph to plot")
        
    # fly to a lat/lon/alt position on the map
    def fly_to(self, lat, lon, alt=0.0):
        global_goal = (lon, lat, alt)
        if self.__is_valid_location(global_goal):
            # set the goal location (start location is always current position)
            self.global_goal_loc = GlobalLocation(global_goal[0], global_goal[1], global_goal[2])
            # start the mission
            self.__start_mission()
        else:
            print("Cannot fly! Invalid goal location...")

    def __start_mission(self):
        
        print("Starting Mission...")

        if self._in_mission is False:

            # start logging
            self.start_log("Logs", "NavLog.txt")

            print("Initializing the connection...")
            if self._connection is None:
                # initiate a connection with 60 sec timeout
                self._connection = MavlinkConnection('tcp:127.0.0.1:5760', threaded=self._threaded, timeout=3000)
            
            super().__init__(self._connection)
            print("... Done!")

            # register all your callbacks here
            self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
            self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
            self.register_callback(MsgID.STATE, self.state_callback)

            self._in_mission = True
            
            print("Starting connection... ")
            self.start()
            print("... Done!")
            if self._threaded:
                print("Blocking in start_mission...")
                while self._end_mission is False:
                    pass
                print("start_mission unblocked...")
            
            self.__end_mission()

        else:
            print("ERROR: Drone already in mission...")
            

    def __end_mission(self):
        if self._end_mission is True:
            # stop logging
            self.stop_log()
            self._in_mission = False
            
    def __is_valid_location(self, location):
        is_valid = True
        
        try:
            assert (len(location) == 3), "Invalid Location. Must be [longitude, latitude, altitude]"
            assert (self._longitude_min <= location[0] <= self._longitude_max), "Invalid Longitude"
            assert (self._latitude_min <= location[1] <= self._latitude_max), "Invalid Latitude"
            assert (location[2] == 0.0), "Invalid Altitude. Altitude (Optional) Must Be 0 (Ground Level)"
        except AssertionError as msg:
            print(msg)
            is_valid = False
                
        return is_valid
                
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            target_alt = self.target_position[3]
            # check whether we are close to target position (N-E-Alt)
            # local_position[2] (depth) is negative altitude, keep altitude proximity to 0.5 meters
            if (np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0 and 
                abs(self.target_position[2] + self.local_position[2]) < 0.5):
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self._in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        print("takeoff position: ", self.target_position)
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        if len(self.waypoints):
            self.target_position = self.waypoints.pop(0)
            print('target position', self.target_position)
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self._end_mission = True
        

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        TAKEOFF_ALTITUDE = self._target_altitude  # 2 # 5
#         SAFETY_DISTANCE = 5

        self.target_position[2] = TAKEOFF_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        # Done in __init__

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(self.global_home_loc.lon, 
                               self.global_home_loc.lat, 
                               self.global_home_loc.alt)
        time.sleep(1)

        # TODO: retrieve current global position
        global_position = self.global_position
 
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
        
        print('self.global_home {0}, self.global_position {1}, self.local_position {2}, calculated local_position {3}'.format(self.global_home, self.global_position,
                                                                         self.local_position, local_position))

        if self._use_grid:
            # Read in obstacle map
#             data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

            # Define a grid for a particular altitude and safety margin around obstacles
            grid, north_offset, east_offset = create_grid(self._data, TAKEOFF_ALTITUDE, SAFETY_DISTANCE)
            print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

            # Define starting point on the grid (this is just grid center)
    #         grid_start = (-north_offset, -east_offset)
            # TODO: convert start position to current position rather than map center
            start_n = int(local_position[0]-north_offset)
            start_e = int(local_position[1]-east_offset)

            grid_start = (start_n, start_e)

            # Set goal as some arbitrary position on the grid
            local_goal = global_to_local(self.global_goal_loc.location, 
                                         self.global_home)

            goal_n = int(local_goal[0]-north_offset)
            goal_e = int(local_goal[1]-east_offset)
            grid_goal = (goal_n, goal_e)
            # TODO: adapt to set goal as latitude / longitude position and convert

            # Run A* to find a path from start to goal
            # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
            # or move to a different search space such as a graph (not done here)
            print('Local Start and Goal: ', grid_start, grid_goal)
            print("Searching for a path ...")
            path, _ = a_star(grid, heuristic, grid_start, grid_goal)

            # TODO: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!

            # Convert path to waypoints
            waypoints = [[p[0] + north_offset, p[1] + east_offset, TAKEOFF_ALTITUDE, 0] for p in path]
#             print("Waypoints: {0}".format(waypoints))
        else: 
            #
            # build a graph of free space
            #
        
            # get start and goal locations. start locations is always current location
            self.global_start_loc = GlobalLocation(self.global_position[0], 
                                                   self.global_position[1], 
                                                   self.global_position[2])

            local_start = global_to_local(self.global_start_loc.location, self.global_home)
            local_goal = global_to_local(self.global_goal_loc.location, self.global_home)

            print("global start:", self.global_start_loc.location, 
                  " global goal:", self.global_goal_loc.location, " local_start: ", local_start, 
                  " local_goal: ", local_goal)

            print("Searching for a path ...")
            waypoints = self._route_planner.get_flight_waypoints(local_start, local_goal, 
                                                                 prune_path=self._prune_path)

            print("Final Waypoints: {0}".format(waypoints))


        print("Num waypoints: ", len(waypoints))
        
        if len(waypoints):
            # Set self.waypoints
            self.waypoints = waypoints
            
            # TODO: send waypoints to sim
            self.send_waypoints()
        else:
            print("No Path found!")

#     def get_ned_waypoint(self, global_loc):
#         return [int(i) for i in global_to_local(global_loc, self.global_home)]

#     def start(self):
#         self.start_log("Logs", "NavLog.txt")

#         print("starting connection")
#         self.connection.start()

#         # Only required if they do threaded
#         # while self.in_mission:
#         #    pass

#         self.stop_log()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('-l', type=float, nargs=2, default=[37.792779, -122.397475], help="Goal Location: lat lon")
#     parser.add_argument('-a', type=int, nargs=1, default=TARGET_ALTITUDE, help="Target Altitude (integer): alt (meters)")
    parser.add_argument('-pg', action='store_true', help="Plots the route graph showing obstacles higher than or equal to target height.")
    args = parser.parse_args()

    alt = TARGET_ALTITUDE
    drone = MotionPlanning(connection=None, target_altitude=alt, safety_distance=SAFETY_DISTANCE,
                           graph_num_samples=ROUTE_GRAPH_NUM_SAMPLES, graph_k=ROUTE_GRAPH_K, use_grid=False,
                           prune_path=True, threaded=False)

    time.sleep(1)

    # plot the route graph is requested
    if args.pg:
        print("Plotting route graph... this may take sometime...")
        drone.plot_route_graph()
        print("...Done")
    else:
        print("Flying Drone...")
        [lat, lon] = args.l
#         alt = args.a

        print("Goal Location: Lat. {0} Lon. {1}".format(lat, lon))
        print("Target Altitude: {0} meters".format(alt))

        drone.fly_to(lat, lon)
    
    
# if __name__ == "__main__":
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--port', type=int, default=5760, help='Port number')
#     parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
#     args = parser.parse_args()

#     conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
#     drone = MotionPlanning(conn)
#     time.sleep(1)

#     drone.start()
