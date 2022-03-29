from abc import ABC, abstractmethod
from operator import le
import random
import os
import sys
import copy
from core.Util import *
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")
import traci
import sumolib

STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"

class RouteController(ABC):
    """
    Base class for routing policy

    To implement a scheduling algorithm, implement the make_decisions() method.
    Please use the boilerplate code from the example, and implement your algorithm between
    the 'Your algo...' comments.

    make_decisions takes in a list of vehicles and network information (connection_info).
        Using this data, it should return a dictionary of {vehicle_id: decision}, where "decision"
        is one of the directions defined by SUMO (see constants above). Any scheduling algorithm
        may be injected into the simulation, as long as it is wrapped by the RouteController class
        and implements the make_decisions method.

    :param connection_info: object containing network information, including:
                            - out_going_edges_dict {edge_id: {direction: out_edge}}
                            - edge_length_dict {edge_id: edge_length}
                            - edge_index_dict {edge_index_dict} keep track of edge ids by an index
                            - edge_vehicle_count {edge_id: number of vehicles at edge}
                            - edge_list [edge_id]

    """
    def __init__(self, connection_info: ConnectionInfo):
        self.connection_info = connection_info
        self.direction_choices = [STRAIGHT, TURN_AROUND,  SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]

    ''' when testing vehicle current speed it always is 0 for some reason, so we assum that the path_length can never exceed 20
    because that is where the while loop is at
    
    this means that if the first choice is given to the compute_local_target is longer than 20, it will disregard other choices and go with that one.
    or the combination of all the paths that are less than 20'''
    def compute_local_target(self, decision_list, vehicle):
        current_target_edge = vehicle.current_edge
        try:
            path_length = 0
            i = 0
            #print("\ninside compute_local_target - vehicle current speed: {}".format(vehicle.current_speed))
            #the while is used to make sure the vehicle will not assume it arrives the destination beacuse the target edge is too short.
            while path_length <= max(vehicle.current_speed, 20):
                
                if current_target_edge == vehicle.destination:
                    break
                if i >= len(decision_list):
                    #print("Throwing userwarning, i:{} - len(decision_list):{}".format(i, len(decision_list)))
                    raise UserWarning(
                        "Not enough decisions provided to compute valid local target. TRACI will remove vehicle."
                    )

                choice = decision_list[i]
                if choice not in self.connection_info.outgoing_edges_dict[current_target_edge]:
                    raise UserWarning(
                            "Invalid direction. TRACI will remove vehicle."
                        )
                current_target_edge = self.connection_info.outgoing_edges_dict[current_target_edge][choice]
                path_length += self.connection_info.edge_length_dict[current_target_edge]
                #print("current_target_edge1:{}".format(current_target_edge))
                #print("Current edge length:{} - path_length:{}".format(self.connection_info.edge_length_dict[current_target_edge], path_length))

                if i > 0:
                    if decision_list[i - 1] == decision_list[i] and decision_list[i] == 't':
                        # stuck in a turnaround loop, let TRACI remove vehicle
                        #print("current_target_edge2:{}".format(current_target_edge))
                        return current_target_edge

                i += 1

        except UserWarning as warning:
            print(warning)

        return current_target_edge


    @abstractmethod
    def make_decisions(self, vehicles, connection_info):
        pass


class RandomPolicy(RouteController):
    """
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """
    def __init__(self, connection_info):
        super().__init__(connection_info)

    def make_decisions(self, vehicles, connection_info):
        """
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """

        local_targets = {}
        for vehicle in vehicles:
            start_edge = vehicle.current_edge
            print("{}: current - {}, destination - {}, deadline - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination, vehicle.deadline))
            '''
            Your algo starts here
            '''
            decision_list = []

            i = 0
            while i < 10:  # choose the number of decisions to make in advanced; depends on the algorithm and network
                choice = self.direction_choices[random.randint(0, 5)]  # 6 choices available in total

                # dead end
                if len(self.connection_info.outgoing_edges_dict[start_edge].keys()) == 0:
                    break

                # make sure to check if it's a valid edge
                if choice in self.connection_info.outgoing_edges_dict[start_edge].keys():
                    decision_list.append(choice)
                    start_edge = self.connection_info.outgoing_edges_dict[start_edge][choice]

                    if i > 0:
                        if decision_list[i-1] == decision_list[i] and decision_list[i] == 't':
                            # stuck in a turnaround loop, let TRACI remove vehicle
                            break

                    i += 1

            '''
            Your algo ends here
            '''
            x = self.compute_local_target(decision_list, vehicle)
            print("compute_local_target returns:{} - with decision list:{}".format(x, decision_list))
            local_targets[vehicle.vehicle_id] = x

        # for vehicle in vehicles:
        #     current_edge = vehicle.current_edge
        #     if current_edge not in self.connection_info.outgoing_edges_dict.keys():
        #         continue
        #     for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
        #         print("Current vehicle: {}".format(vehicle.vehicle_id))
        #         print("current edge: {} - direction: {} - edge:{}".format(current_edge,direction, outgoing_edge))
        #         print("Vehicles on the potential edge: {}".format(self.connection_info.edge_vehicle_count[outgoing_edge]))
        #     print("\n")

        return local_targets


class NathanPolicy(RouteController):
    def __init__(self, connection_info):
        super().__init__(connection_info)
    
    def make_decisions(self, vehicles, connection_info):
        """
        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """
        local_targets = {}

        if len(vehicles) == 0:
            return local_targets


        # create edge_vehicle { edge_id : [vehicle OBJECT]} to keep track of all the vehicles that on the same edge
        edge_vehicle = {}
        for vehicle in vehicles:
            if vehicle.current_edge not in self.connection_info.outgoing_edges_dict.keys():
                continue
            if vehicle.current_edge in edge_vehicle:
                edge_vehicle[vehicle.current_edge].append(vehicle)
            else:
                edge_vehicle[vehicle.current_edge] = list()
                edge_vehicle[vehicle.current_edge].append(vehicle)
        # sort all vehicles by their deadline in edge_vehicles values 
        for edge, v_list in edge_vehicle.items():
            v_list.sort(key=lambda x: x.deadline)


        #create vehicle_decisionList {vehicle OBJECT : [decision list]} because instead of sending decision list to the
        #local target we wait to check for congestion
        vehicle_decisionList = {}
        for edge, vehicles in edge_vehicle.items():
            for vehicle in vehicles:
                #instantiate a list of decision for each vehicle
                vehicle_decisionList[vehicle] = list()

        # --- start dijkstra ---
        # apply dijkstra to each vehicle in edge_vehicle DS
        for edge, vehicles in edge_vehicle.items():
            #below is copied from dijkstra 
            for vehicle in vehicles:
                #print("---------------------------")
                #print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
                decision_list = []
                unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges {edge_id : distance}
                visited = {} # map of visited edges
                current_edge = vehicle.current_edge
                #print("vehicle current_edge length:{}".format(self.connection_info.edge_length_dict[current_edge]))

                #print("\n------- getting into while loop ----------\n")
                #vehicle is at the beginning of the edge so current edge length counts too
                current_distance = self.connection_info.edge_length_dict[current_edge]
                unvisited[current_edge] = current_distance
                #stores shortest path to each edge using directions [edge_id]
                path_lists = {edge: [] for edge in self.connection_info.edge_list} 
                
                while True:
                    if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                        continue
                    for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                        #print("----- in for loop -----")
                        if outgoing_edge not in unvisited:
                            continue
                        edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                        new_distance = current_distance + edge_length
                        #univisited length are set to 100000 so this is almost always true 
                        if new_distance < unvisited[outgoing_edge]:
                            '''
                            print("\nchecking direction:{} - edge:{}".format(direction, outgoing_edge))
                            print("new distance < outgoing_edge length")
                            print("new_distance = current_distance + edge_length = {}".format(new_distance))
                            print("unvisited[outgoing_edge]: {}".format(unvisited[outgoing_edge]))
                            '''
                            #distance to get to this 'outgoing_edge' is now 'new_distance'
                            unvisited[outgoing_edge] = new_distance

                            #path_list[current_edge] is initially set to empty array
                            #so we assgin current_path to that empty array so we can modify it
                            current_path = copy.deepcopy(path_lists[current_edge])
                            
                            #append this new shortest path to current_edge; example: path_list[E9] = [s , l , s , r] 
                            #[s, l , s , r] is current_path
                            current_path.append(direction)
                            # reassign path_list[E9] to current path  [s,l,s,r]
                            path_lists[outgoing_edge] = copy.deepcopy(current_path)
                            #print("current path_list[{}]:{}".format(outgoing_edge, path_lists[outgoing_edge]))
                            # NOT nathan print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))
                    #print("---- out for loop ----")
                    visited[current_edge] = current_distance
                    del unvisited[current_edge]
                    if not unvisited:
                        break
                    if current_edge==vehicle.destination:
                        break
                    #update possible_edge every time we modify - del unvisited above
                    possible_edges = [edge for edge in unvisited.items() if edge[1]]

                    #sort by x[1] meaning distance, and [0] meaning we take the smallest distance
                    #and set our current_edge to that value
                    current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
                    '''
                    print("\npossible edge:{}".format(possible_edges))
                    print("\ncurrent_edge:{}".format(current_edge))
                    print("current_edge length:{}".format(self.connection_info.edge_length_dict[current_edge]))
                    print("current_distance:{}".format(current_distance))
                    print("\n ---------- repeating while loop ----------------\n")
                    #print('{}:{}------------'.format(current_edge, current_distance))
                    '''
                #current_edge = vehicle.current_edge - not NATHAN
                #print("\nout of while true loop -----")
                #print("\npath_list:{}".format(path_lists))
                for direction in path_lists[vehicle.destination]:
                    decision_list.append(direction)
                #print("\ndecision_list:{}".format(decision_list))
                vehicle_decisionList[vehicle] = decision_list

        #----- outside dijkstra method------
        # print("\n ---- testing vehicle and destination list:")
        # for vehicle, decisionList in vehicle_decisionList.items():
        #     print("vehicle:{} - decision_list:{}".format(vehicle.vehicle_id, decisionList))

        '''
        check if route vehicle is gonna take is full ( count > 2)

        if yes then create a dict outEdge_count {direction : count}
        then sort by count on that dict 
        then send the lowest one to decision list 
        and append that to local target

        if no, then append like dijkstra would have 
        '''
        for vehicle, decision_list in vehicle_decisionList.items():
            if len(decision_list) == 0:
                local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
                continue            
            #the next 3 lines is used to check if vehicle next item is its destination then we dont need to do any work on it
            direction_first_item_in_decision_list = decision_list[0]
            edge_first_item_in_decision_list = self.connection_info.outgoing_edges_dict[vehicle.current_edge][direction_first_item_in_decision_list]
            if edge_first_item_in_decision_list == vehicle.destination:
                local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
                continue

            #getting the next edge id 
            nextEdge_id = self.connection_info.outgoing_edges_dict[vehicle.current_edge][decision_list[0]]
            choices_available = len(self.connection_info.outgoing_edges_dict[vehicle.current_edge])
            #if the next edge in the vehicle is too crowded (4) and there are other choices available, then we re-route the vehicle
            if self.connection_info.edge_vehicle_count[nextEdge_id] >= 10 and choices_available > 1 and len(decision_list) > 4:
                #gather all the choices available and send to the smallest count choice
                outEdgeDirection_count = {}
                #explore all the route available to the vehicle in outgoing_edges_dict
                for direction, outEdge_id in self.connection_info.outgoing_edges_dict[vehicle.current_edge].items():
                    #if the 'possible' edge that we are sending the vehicle to is a dead end (no routes available in outgoing_edge)
                    # then we dont add (meaning if len is not 0 then we add)
                    if len(self.connection_info.outgoing_edges_dict[outEdge_id].items()) != 0:
                        outEdgeDirection_count[direction] = self.connection_info.edge_vehicle_count[outEdge_id]
                outEdgeDirection_count = dict(sorted(outEdgeDirection_count.items(), key=lambda item: item[1]))

                
                # test sort
                #print("\n-- testing outEdge_count sort:{}\n".format(outEdge_count))
                new_list =  list()
                new_list.append(list(outEdgeDirection_count.keys())[0])
                # print("\nold_list:{}".format(decision_list))
                # print("\nnew list:{}".format(new_list))
                local_targets[vehicle.vehicle_id] = self.compute_local_target(new_list, vehicle)

            else:
                local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        # print("------\nlocal_targets:{}".format(local_targets))
        return local_targets 