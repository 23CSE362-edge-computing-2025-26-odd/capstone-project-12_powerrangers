import time
import traci
import math

class CENBroadcast:
    def __init__(self, interval=5, edge_nodes=None):
        self.interval = interval
        self.accidents = {}
        self.cleared_accidents = set()  # Track cleared accidents
        self.edge_nodes = edge_nodes if edge_nodes else {}

        self.edgecen_names = {
            "EdgeNode_A": "EdgeCEN_A",
            "EdgeNode_D": "EdgeCEN_D",
            "EdgeNode_C": "EdgeCEN_C",
            "EdgeNode_I": "EdgeCEN_I"
        }

    def get_nearest_edge_node(self, position):
        x, y = position
        min_dist = float('inf')
        nearest_edge = None
        for edge_id, info in self.edge_nodes.items():
            node_x, node_y = info['position']
            dist = math.hypot(x - node_x, y - node_y)
            if dist < min_dist:
                min_dist = dist
                nearest_edge = edge_id
        return nearest_edge

    def register(self, accident_id, location, sim_time, vehicles_involved, accident_edge=None):
        """
        Register a new accident with the nearest CEN.
        vehicles_involved: list of vehicle IDs involved in this accident
        accident_edge: the SUMO edge ID where the accident occurred
        """
        registering_edge = self.get_nearest_edge_node(location)
        registering_name = self.edgecen_names.get(registering_edge, registering_edge)

        self.accidents[accident_id] = {
            "location": location,
            "last_time": sim_time,
            "registered_by_edge": registering_edge,
            "registered_by_name": registering_name,
            "vehicles_involved": set(vehicles_involved) if isinstance(vehicles_involved, list) else {vehicles_involved},
            "edge": accident_edge,  # Store the actual SUMO edge where accident occurred
            "cleared": False  # Track if accident has been cleared
        }

        print(f"[CEN {registering_name} REGISTER] Accident {accident_id} at {location} on edge {accident_edge}")
        print(f"[CEN {registering_name} REGISTER] Vehicles involved: {vehicles_involved} (sim time {sim_time:.1f}s)")

    def mark_accident_cleared(self, accident_id):
        """Mark an accident as cleared - stop broadcasting it"""
        if accident_id in self.accidents:
            self.accidents[accident_id]['cleared'] = True
            self.cleared_accidents.add(accident_id)
            print(f"[CEN] Accident {accident_id} marked as CLEARED - will stop broadcasting")

    def broadcast(self, sim_time, vehicles_dict, graph, comm_range=None):
        """Broadcast accident info to nearby edge nodes and vehicles"""
        for acc_id, data in self.accidents.items():
            # CRITICAL: Skip broadcasting if accident is cleared
            if data.get('cleared', False):
                continue
            
            if acc_id in self.cleared_accidents:
                continue
            
            if sim_time - data["last_time"] >= self.interval:
                broadcasting_name = data['registered_by_name']
                broadcasting_edge = data['registered_by_edge']
                accident_edge = data.get('edge', None)

                print(f"[CEN {broadcasting_name} BROADCAST] Accident {acc_id} on edge {accident_edge} being broadcast at t={sim_time:.1f}s")

                # --- Notify edge nodes ---
                if self.edge_nodes:
                    for edge_id, edge_info in self.edge_nodes.items():
                        edge_pos = edge_info['position']
                        distance = math.hypot(edge_pos[0] - data['location'][0], edge_pos[1] - data['location'][1])
                        if distance <= comm_range:
                            receiver_name = self.edgecen_names.get(edge_id, edge_id)
                            print(f"    [CEN {receiver_name} RECEIVED] Accident {acc_id} on edge {accident_edge} (distance: {distance:.1f})")

                # --- Notify vehicles in range (skip involved vehicles) ---
                if vehicles_dict and self.edge_nodes:
                    positions_dict = {k: v['position'] for k, v in self.edge_nodes.items()}
                    for vid, vehicle in vehicles_dict.items():
                        # Skip vehicles involved in THIS accident
                        if vid in data['vehicles_involved']:
                            continue
                        
                        try:
                            veh_pos = traci.vehicle.getPosition(vid)
                            cen_pos = self.edge_nodes[broadcasting_edge]['position']
                            dist = math.hypot(veh_pos[0] - cen_pos[0], veh_pos[1] - cen_pos[1])
                            
                            if dist <= comm_range:
                                # Let vehicle handle the rerouting
                                vehicle.listen_and_reroute(
                                    cen=self,
                                    cen_positions=positions_dict,
                                    graph=graph,
                                    comm_range=comm_range
                                )
                        except:
                            continue

                data["last_time"] = sim_time