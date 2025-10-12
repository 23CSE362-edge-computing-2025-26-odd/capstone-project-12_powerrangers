# vehicle.py
import math
import traci
import random
from collections import deque

class Vehicle:
    def __init__(self, veh_id, destination):
        self.veh_id = veh_id
        self.destination = destination
        self.current_edge = None
        self.route = []
        self.accidents_received = set()
        self.is_in_accident = False

    def mark_as_accident_vehicle(self):
        """Mark this vehicle as involved in an accident"""
        self.is_in_accident = True
        print(f"[VEHICLE] {self.veh_id} marked as accident vehicle - will not reroute")

    def update_position(self):
        try:
            self.current_edge = traci.vehicle.getRoadID(self.veh_id)
            self.route = list(traci.vehicle.getRoute(self.veh_id))
        except:
            pass

    def distance_to_cen(self, cen_pos):
        try:
            veh_pos = traci.vehicle.getPosition(self.veh_id)
            return math.hypot(veh_pos[0] - cen_pos[0], veh_pos[1] - cen_pos[1])
        except:
            return float('inf')

    def listen_and_reroute(self, cen, cen_positions, graph, comm_range=200):
        # CRITICAL: Skip if this vehicle is involved in an accident
        if self.is_in_accident:
            return
        
        # Check if vehicle is in any accident's vehicles_involved set
        for acc_id, data in cen.accidents.items():
            vehicles_involved = data.get("vehicles_involved", set())
            if self.veh_id in vehicles_involved:
                if not self.is_in_accident:
                    self.is_in_accident = True
                    print(f"[VEHICLE] {self.veh_id} detected as accident vehicle via CEN, will not reroute")
                return
        
        self.update_position()
        try:
            veh_pos = traci.vehicle.getPosition(self.veh_id)
        except:
            return

        for acc_id, data in cen.accidents.items():
            if acc_id in self.accidents_received:
                continue

            # Check if this vehicle is involved in THIS accident
            vehicles_involved = data.get("vehicles_involved", set())
            if self.veh_id in vehicles_involved:
                print(f"[REROUTE] Vehicle {self.veh_id} is involved in accident {acc_id}, skipping reroute")
                self.accidents_received.add(acc_id)
                continue

            broadcasting_cen_name = data["registered_by_name"]
            broadcasting_edge = data["registered_by_edge"]

            if broadcasting_edge not in cen_positions:
                continue

            dist = self.distance_to_cen(cen_positions[broadcasting_edge])
            if dist <= comm_range:
                self.accidents_received.add(acc_id)

                print(f"[V2I] Vehicle {self.veh_id} received accident {acc_id} info from CEN {broadcasting_cen_name} "
                      f"at t={traci.simulation.getTime():.1f}s (distance {dist:.1f})")

                # Get the accident edge from CEN data
                accident_edge = data.get("edge", None)
                
                if not accident_edge:
                    print(f"[REROUTE] Vehicle {self.veh_id} - No accident edge info, skipping")
                    continue
                
                print(f"[REROUTE] Vehicle {self.veh_id} - Accident on edge: {accident_edge}")
                print(f"[REROUTE] Vehicle {self.veh_id} - Current edge: {self.current_edge}")
                print(f"[REROUTE] Vehicle {self.veh_id} - Current route: {self.route}")
                
                # Check if accident edge is in our future route
                try:
                    current_idx = self.route.index(self.current_edge)
                    remaining_route = self.route[current_idx:]
                    
                    if accident_edge not in remaining_route:
                        print(f"[REROUTE] Vehicle {self.veh_id} - Accident edge NOT in remaining route")
                        continue
                    
                    print(f"[REROUTE] Vehicle {self.veh_id} - Accident edge IS in remaining route, computing alternative with BFS...")
                    
                except ValueError:
                    if accident_edge not in self.route:
                        print(f"[REROUTE] Vehicle {self.veh_id} - Accident edge NOT in route")
                        continue
                    remaining_route = self.route
                
                old_route = list(self.route)
                new_route = self.find_path_bfs_manual(self.current_edge, self.route[-1], accident_edge)
                
                if new_route and new_route != old_route and accident_edge not in new_route:
                    try:
                        traci.vehicle.setRoute(self.veh_id, new_route)
                        self.route = list(new_route)
                        print(f"[REROUTE SUCCESS] Vehicle {self.veh_id}")
                        print(f"  OLD: {old_route}")
                        print(f"  NEW: {new_route}")
                        print(f"  Blocked edge {accident_edge} successfully avoided!")
                    except traci.exceptions.TraCIException as e:
                        print(f"[ERROR] Failed to set route for {self.veh_id}: {e}")
                else:
                    if new_route and accident_edge in new_route:
                        print(f"[REROUTE FAILED] Vehicle {self.veh_id} - New route still contains blocked edge!")
                    else:
                        print(f"[REROUTE SKIP] Vehicle {self.veh_id} - No alternative route found")

    def find_path_bfs_manual(self, start_edge, end_edge, blocked_edge):
        """Manual BFS using edge connectivity from SUMO"""
        print(f"[BFS] Finding path from {start_edge} to {end_edge}, avoiding {blocked_edge}")
        
        if start_edge == end_edge:
            return [start_edge]
        
        if start_edge == blocked_edge:
            print(f"[BFS] Start edge is blocked!")
            return None
        
        def get_outgoing_edges(edge_id):
            """Get all edges that can follow this edge"""
            outgoing = []
            try:
                # Try SUMO's lane links first
                lane_id = edge_id + "_0"
                links = traci.lane.getLinks(lane_id)
                
                for link in links:
                    next_lane = link[0]
                    if next_lane:
                        next_edge = next_lane.rsplit('_', 1)[0]
                        if not next_edge.startswith(':') and 'parking' not in next_edge:
                            outgoing.append(next_edge)
                
                if outgoing:
                    return outgoing
            except:
                pass
            
            # Fallback: parse edge name
            parts = edge_id.split('_')
            if len(parts) >= 2:
                to_node = parts[-1]  # Destination node of current edge
                
                # Get all edges from SUMO
                all_edges = traci.edge.getIDList()
                for other_edge in all_edges:
                    if other_edge.startswith(':') or 'parking' in other_edge:
                        continue
                    
                    other_parts = other_edge.split('_')
                    if len(other_parts) >= 2:
                        # Check if other_edge starts from our destination node
                        if other_parts[0] == to_node:
                            outgoing.append(other_edge)
            
            return outgoing
        
        # BFS
        queue = deque([(start_edge, [start_edge])])
        visited = {start_edge}
        
        max_iterations = 200
        iterations = 0
        
        while queue and iterations < max_iterations:
            iterations += 1
            current_edge, path = queue.popleft()
            
            # Get neighbors
            neighbors = get_outgoing_edges(current_edge)
            
            for next_edge in neighbors:
                # Skip blocked and visited
                if next_edge == blocked_edge or next_edge in visited:
                    continue
                
                new_path = path + [next_edge]
                
                # Found destination?
                if next_edge == end_edge:
                    print(f"[BFS SUCCESS] Found path in {iterations} iterations: {' -> '.join(new_path)}")
                    return new_path
                
                visited.add(next_edge)
                queue.append((next_edge, new_path))
        
        print(f"[BFS FAILED] No path found in {iterations} iterations, visited {len(visited)} edges")
        return None



