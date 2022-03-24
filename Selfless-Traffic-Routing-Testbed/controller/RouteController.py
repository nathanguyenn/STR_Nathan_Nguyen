from abc import ABC, abstractmethod
from operator import le
import random
import os
import sys
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
            print("\ninside compute_local_target - vehicle current speed: {}".format(vehicle.current_speed))
            #the while is used to make sure the vehicle will not assume it arrives the destination beacuse the target edge is too short.
            while path_length <= max(vehicle.current_speed, 20):
                
                if current_target_edge == vehicle.destination:
                    break
                if i >= len(decision_list):
                    print("Throwing userwarning, i:{} - len(decision_list):{}".format(i, len(decision_list)))
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
                print("current_target_edge1:{}".format(current_target_edge))
                print("Current edge length:{} - path_length:{}".format(self.connection_info.edge_length_dict[current_target_edge], path_length))

                if i > 0:
                    if decision_list[i - 1] == decision_list[i] and decision_list[i] == 't':
                        # stuck in a turnaround loop, let TRACI remove vehicle
                        print("current_target_edge2:{}".format(current_target_edge))
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
        through realization i know that all vehicles that come into this parameter NEED to make the decision
        so all their current_edge are inside outgoing_edges in connection_info


        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """
        local_targets = {}


        if len(vehicles) == 0:
            return local_targets
        print("\n-------------------------")
        print("All vehicle present:")
        vehicle_deadline = {}
        for vehicle in vehicles:
            vehicle_deadline[vehicle.vehicle_id] = vehicle.deadline
            print("id: {} - current edge:{} - deadline:{}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.deadline))

        # vehicle_deadline = dict(sorted(vehicle_deadline.items(), key=lambda item: item[1]))

        # create dict { edge_id : [vehicle]} to keep track of all the vehicles that on the same edge
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

        # create a dict{current_edge : [out_edges]} 
        # we create working_edges to get all the edges containing the vehicles we are working with
        # then currentEdge_potentialEdges is the dict that points to all the potential edges from our current_edge
        working_edges = list(edge_vehicle.keys())
        currentEdge_potentialEdges = {}
        for edge in working_edges:
            currentEdge_potentialEdges[edge] = list()

        for edge in working_edges:
            for direction, out_edge in self.connection_info.outgoing_edges_dict[edge].items():
                currentEdge_potentialEdges[edge].append(out_edge)
        # done - now need to sort out_edge by their length and count

        currentEdge_potentialEdges2 = {}
        for edge in working_edges:
            currentEdge_potentialEdges2[edge] = list()    

        for edge , out_edge in currentEdge_potentialEdges.items():
            for outEdge in out_edge:
                length = self.connection_info.edge_length_dict[outEdge]
                count = self.connection_info.edge_vehicle_count[outEdge]
                xample = Edge(outEdge, length, count)
                currentEdge_potentialEdges2[edge].append(xample)

        #testing
        for edge, out_edge in currentEdge_potentialEdges2.items():
            #confirmed sort by length, then count
            out_edge.sort(key=lambda x: x.vehicle_count)
            out_edge.sort(key=lambda x: x.length)
            

        for edge, vehicle_list in edge_vehicle.items():
            for vehicle in vehicle_list:
                hold = currentEdge_potentialEdges2[edge][0]
                print("current vehicle id:{} - target_edge_id:{}".format(vehicle.vehicle_id, hold.id))
                local_targets[vehicle.vehicle_id] = hold.id
                currentEdge_potentialEdges2[edge].sort(key=lambda x: x.vehicle_count)
                currentEdge_potentialEdges2[edge].sort(key=lambda x: x.length)



 
        # possible_choice = {}
        # for direction, target_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
        #     possible_choice[target_edge] = direction
        
        # vehicle_count = {}
        # for edge in possible_choice.keys():
        #     count = self.connection_info.edge_vehicle_count[edge]
        #     vehicle_count[edge]= count
        #     print("Adding edge:{} - count:{}".format(edge,count))




        # for vehicle in vehicles:
        #     decision_list = []
        #     current_edge = vehicle.current_edge
        #     for choice in self.connection_info.outgoing_edges_dict[current_edge].keys():
        #         decision_list.append(choice)

        #     x = self.compute_local_target(decision_list, vehicle)
        #     print("compute_local_target returns:{} - with decision list:{}".format(x, decision_list))
        #     local_targets[vehicle.vehicle_id] = x                
        #     # local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets    

class Edge:
    def __init__(self, id, length, vehicle_count):
        self.id = id
        self.length = length
        self.vehicle_count = vehicle_count