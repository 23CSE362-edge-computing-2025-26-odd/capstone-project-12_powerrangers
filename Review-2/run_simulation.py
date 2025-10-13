import sys
import os
import time
import random
import xml.etree.ElementTree as ET
import math
import uuid
from collections import defaultdict
from cen_broadcast import CENBroadcast
from vehicle import Vehicle
import traci
import sumolib
# -------------------- CLOUD INTEGRATION START --------------------
# Install the required library first: pip install supabase
from supabase import create_client, Client
# -------------------- CLOUD INTEGRATION END --------------------


# -------------------- SUMO PATH SETUP --------------------
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)
from best_erv import select_best_ambulance

# -------------------- SUPABASE SETUP --------------------
SUPABASE_URL = "https://viktldenjhwndygewykg.supabase.co"
SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InZpa3RsZGVuamh3bmR5Z2V3eWtnIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTk5MzQ0MjYsImV4cCI6MjA3NTUxMDQyNn0.JYjoyhYFeB6kbWD0vBD_Tmn7-CcpVGz08HgaIt3eM20"
TABLE = "accidents"

# Initialize Supabase client
try:
    supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)
    print("[CLOUD] Successfully connected to Supabase.")
except Exception as e:
    supabase = None
    print(f"[CLOUD ERROR] Could not connect to Supabase: {e}")

# -------------------- CLOUD LOGGING FUNCTIONS --------------------
def log_new_accident_to_cloud(accident_data):
    if not supabase:
        print("[CLOUD] Supabase client not available. Skipping log.")
        return
    try:
        data, count = supabase.table(TABLE).insert(accident_data).execute()
        print(f"[CLOUD] Logged new accident: {accident_data['accident_id']}")
    except Exception as e:
        print(f"[CLOUD ERROR] Failed to log new accident {accident_data['accident_id']}: {e}")

def update_accident_in_cloud(accident_id, update_data):
    if not supabase:
        print("[CLOUD] Supabase client not available. Skipping update.")
        return
    try:
        data, count = supabase.table(TABLE).update(update_data).eq('accident_id', accident_id).execute()
        print(f"[CLOUD] Updated accident {accident_id} with status: {update_data.get('status', 'N/A')}")
    except Exception as e:
        print(f"[CLOUD ERROR] Failed to update accident {accident_id}: {e}")


# -------------------- SUMO SETUP --------------------
sumoBinary = sumolib.checkBinary("sumo-gui")
sumoConfig = "simulation.sumocfg"
vehicles_dict = {}
V2V_COMMUNICATION_RANGE = 200.0
MAX_HOP_COUNT = 5
COLLISION_DISTANCE = 7.5

# Edge node positions
EDGE_NODE_POSITIONS = {
    "EdgeNode_A": (30, 20),
    "EdgeNode_D": (30, 120),
    "EdgeNode_C": (200,180),
    "EdgeNode_I": (200,20)
}

# Global state
edge_nodes = {}
broadcasted_accidents = set()
message_history = {}
hop_count_stats = defaultdict(int)
reported_collisions = set()
total_accidents = 0
successful_notifications = 0

# ERV MANAGEMENT STATE
erv_assignments = {}
erv_status = {}
erv_destinations = {}
erv_target_edge = {}
erv_optimal_route = {}  # NEW: Store the optimal route for each ERV
accident_status = {}
erv_arrival_times = {}
accident_vehicles = {}

# -------------------- PARSE ROUTES --------------------
rou_file = "vehicles.rou.xml"
tree = ET.parse(rou_file)
root = tree.getroot()
routes = [r.attrib['id'] for r in root.findall("route") if not r.attrib['id'].startswith('routeAmbulance')]
vtypes = {v.attrib['id']: v for v in root.findall("vType") if v.attrib['id'] != 'ambulance'}

NUM_INITIAL_VEHICLES = 4
SPAWN_INTERVAL = 20
VEHICLE_COUNTER = 0

# -------------------- V2V MESSAGE CLASS --------------------
class V2VMessage:
    def __init__(self, message_id, source_id, message_type, payload, origin_location):
        self.message_id = message_id
        self.source_id = source_id
        self.message_type = message_type
        self.payload = payload
        self.origin_location = origin_location
        self.hop_count = 0
        self.propagation_path = [source_id]
        self.timestamp = traci.simulation.getTime()
        self.reached_edge_node = False

    def increment_hop(self, next_node):
        self.hop_count += 1
        self.propagation_path.append(next_node)

# -------------------- INITIALIZE EDGE NODES --------------------
def initialize_edge_nodes():
    for node_id, position in EDGE_NODE_POSITIONS.items():
        edge_nodes[node_id] = {
            'position': position,
            'connected_vehicles': set(),
            'message_cache': set(),
            'total_messages_received': 0,
            'unique_accidents_reported': set(),
            'accident_alerts_received': []
        }

# -------------------- HELPER FUNCTIONS --------------------
def get_vehicle_position(vehicle_id):
    try:
        if vehicle_id in traci.vehicle.getIDList():
            return traci.vehicle.getPosition(vehicle_id)
    except:
        pass
    return None

def calculate_distance(pos1, pos2):
    if pos1 and pos2:
        return math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
    return float('inf')

def find_vehicles_in_range(source_position, exclude_vehicle=None):
    vehicles_in_range = []
    for vehicle_id in traci.vehicle.getIDList():
        if vehicle_id == exclude_vehicle or vehicle_id.startswith('ambulance'):
            continue
        vehicle_pos = get_vehicle_position(vehicle_id)
        if vehicle_pos:
            distance = calculate_distance(source_position, vehicle_pos)
            if distance <= V2V_COMMUNICATION_RANGE:
                vehicles_in_range.append((vehicle_id, distance, vehicle_pos))
    vehicles_in_range.sort(key=lambda x: x[1])
    return vehicles_in_range

def find_edge_nodes_in_range(position):
    edge_nodes_in_range = []
    for edge_id, edge_info in edge_nodes.items():
        distance = calculate_distance(position, edge_info['position'])
        if distance <= V2V_COMMUNICATION_RANGE:
            edge_nodes_in_range.append((edge_id, distance))
    edge_nodes_in_range.sort(key=lambda x: x[1])
    return edge_nodes_in_range

def propagate_v2v_message(message, current_vehicle_id, current_position):
    if message.hop_count >= MAX_HOP_COUNT:
        return False

    edge_nodes_in_range = find_edge_nodes_in_range(current_position)
    for edge_id, distance in edge_nodes_in_range:
        if message.message_id not in edge_nodes[edge_id]['message_cache']:
            edge_nodes[edge_id]['message_cache'].add(message.message_id)
            edge_nodes[edge_id]['total_messages_received'] += 1
            if message.message_type == "EMERGENCY":
                edge_nodes[edge_id]['unique_accidents_reported'].add(message.payload.get('accident_id'))
                alert_info = {
                    'accident_id': message.payload.get('accident_id'),
                    'vehicles_involved': message.payload.get('vehicles_involved', []),
                    'location': message.payload.get('location'),
                    'timestamp': message.payload.get('timestamp'),
                    'received_at': traci.simulation.getTime(),
                    'hop_count': message.hop_count,
                    'propagation_path': message.propagation_path.copy()
                }
                edge_nodes[edge_id]['accident_alerts_received'].append(alert_info)
            message.reached_edge_node = True
            hop_count_stats[message.hop_count] += 1
            return True

    vehicles_in_range = find_vehicles_in_range(current_position, exclude_vehicle=current_vehicle_id)
    propagation_successful = False
    for vehicle_id, distance, vehicle_pos in vehicles_in_range:
        if vehicle_id in message.propagation_path:
            continue
        next_message = V2VMessage(
            message.message_id,
            vehicle_id,
            message.message_type,
            message.payload,
            message.origin_location
        )
        next_message.hop_count = message.hop_count + 1
        next_message.propagation_path = message.propagation_path.copy()
        next_message.increment_hop(vehicle_id)
        if next_message.message_id not in message_history:
            message_history[next_message.message_id] = []
        message_history[next_message.message_id].append(next_message)
        success = propagate_v2v_message(next_message, vehicle_id, vehicle_pos)
        if success:
            propagation_successful = True
            break
    return propagation_successful

def broadcast_emergency_alert(source_vehicle_id, accident_location, collision_pair, accident_id):
    global successful_notifications
    source_position = get_vehicle_position(source_vehicle_id)
    if not source_position:
        return False
    message_id = f"EMERGENCY_{uuid.uuid4().hex[:8]}"
    emergency_payload = {
        'accident_id': accident_id,
        'vehicles_involved': collision_pair,
        'location': accident_location,
        'severity': 'HIGH',
        'timestamp': traci.simulation.getTime()
    }
    message = V2VMessage(
        message_id,
        source_vehicle_id,
        "EMERGENCY",
        emergency_payload,
        accident_location
    )
    message_history[message_id] = [message]
    success = propagate_v2v_message(message, source_vehicle_id, source_position)
    if success:
        successful_notifications += 1
    return success

def detect_accident(veh1, veh2, x, y):
    """Detect accident edge from vehicles or fallback to position"""
    for v in [veh1, veh2]:
        try:
            if v in traci.vehicle.getIDList():
                edge = traci.vehicle.getRoadID(v)
                if edge and edge != "" and not edge.startswith(':'):
                    print(f"[ACCIDENT] Edge: {edge} (from {v})")
                    return edge
        except:
            continue
    
    try:
        all_edges = traci.edge.getIDList()
        min_dist = float('inf')
        closest_edge = None
        
        for edge_id in all_edges:
            if edge_id.startswith(':') or 'parking' in edge_id.lower():
                continue
            
            try:
                shape = traci.edge.getShape(edge_id)
                for shape_point in shape:
                    dist = math.hypot(x - shape_point[0], y - shape_point[1])
                    if dist < min_dist:
                        min_dist = dist
                        closest_edge = edge_id
            except:
                continue
        
        if closest_edge and min_dist < 50.0:
            print(f"[ACCIDENT] Edge: {closest_edge} (nearest, {min_dist:.1f}m)")
            return closest_edge
    except:
        pass
    
    print(f"[ACCIDENT] Edge: None (could not detect)")
    return None

# -------------------- ERV FUNCTIONS --------------------
def initialize_erv_state(erv_id):
    if erv_id not in erv_status:
        erv_status[erv_id] = 'idle'
        erv_assignments[erv_id] = None
        erv_destinations[erv_id] = None
        erv_target_edge[erv_id] = None
        erv_optimal_route[erv_id] = None
        erv_arrival_times[erv_id] = None

def get_available_erv(ambulance_keys):
    for erv_id in ambulance_keys:
        if erv_status.get(erv_id, 'idle') == 'idle':
            return erv_id
    return None

def assign_erv_to_accident(erv_id, accident_id, accident_x, accident_y, accident_edge, collision_pair, optimal_route):
    """Assign ERV with its optimal route"""
    erv_assignments[erv_id] = accident_id
    erv_status[erv_id] = 'assigned'
    erv_destinations[erv_id] = (accident_x, accident_y)
    erv_target_edge[erv_id] = accident_edge
    erv_optimal_route[erv_id] = optimal_route  # Store the optimal route
    accident_status[accident_id] = {
        'location': (accident_x, accident_y),
        'erv': erv_id,
        'cleared': False,
        'vehicles': list(collision_pair),
        'detection_time': traci.simulation.getTime() # Store detection time
    }
    accident_vehicles[accident_id] = list(collision_pair)
    print(f"[ERV] {erv_id} assigned to {accident_id} on edge {accident_edge}")

    # --- CLOUD LOG ---
    assignment_time = traci.simulation.getTime()
    update_payload = {
        "assigned_ambulance": erv_id,
        "assignment_time": assignment_time,
        "status": "assigned"
    }
    update_accident_in_cloud(accident_id, update_payload)
    # --- END CLOUD LOG ---

def dispatch_erv(erv_id, accident_edge, accident_location):
    """Dispatch ERV using the pre-computed optimal route"""
    if erv_id not in traci.vehicle.getIDList():
        return False
    
    try:
        traci.vehicle.setStop(erv_id, edgeID="", pos=0, duration=0, flags=0)
    except:
        pass
    
    try:
        traci.vehicle.setMaxSpeed(erv_id, 30.0)
        traci.vehicle.setSpeed(erv_id, -1)
    except:
        pass
    
    # Use the optimal route computed by GA
    optimal_route = erv_optimal_route.get(erv_id)
    
    if optimal_route and len(optimal_route) > 0:
        try:
            traci.vehicle.setRoute(erv_id, optimal_route)
            print(f"[ERV] {erv_id} dispatched with optimal route ({len(optimal_route)} edges)")
            erv_status[erv_id] = 'en_route'
            return True
        except Exception as e:
            print(f"[ERV] Failed to set optimal route: {e}")
    
    # Fallback to direct routing
    if accident_edge:
        try:
            current_edge = traci.vehicle.getRoadID(erv_id)
            if current_edge and current_edge != accident_edge:
                route_edges = traci.simulation.findRoute(current_edge, accident_edge)
                if route_edges and route_edges.edges:
                    traci.vehicle.setRoute(erv_id, list(route_edges.edges))
                    print(f"[ERV] {erv_id} using fallback route")
                else:
                    traci.vehicle.setRoute(erv_id, [accident_edge])
            erv_status[erv_id] = 'en_route'
            return True
        except Exception as e:
            print(f"[ERV] Dispatch error: {e}")
            return False
    return False

def check_erv_arrival(erv_id, accident_id):
    if erv_id not in traci.vehicle.getIDList() or accident_id not in accident_status:
        return False
    
    try:
        erv_pos = traci.vehicle.getPosition(erv_id)
        acc_x, acc_y = accident_status[accident_id]['location']
        distance = calculate_distance(erv_pos, (acc_x, acc_y))
        current_edge = traci.vehicle.getRoadID(erv_id)
        target_edge = erv_target_edge.get(erv_id)
        
        if distance <= 20.0 or (target_edge and current_edge == target_edge):
            arrival_time = traci.simulation.getTime()
            erv_status[erv_id] = 'arrived'
            erv_arrival_times[erv_id] = arrival_time
            print(f"[ERV] {erv_id} arrived at {accident_id}")
            
            # --- CLOUD LOG ---
            update_payload = {
                "arrival_time": arrival_time,
                "status": "arrived"
            }
            update_accident_in_cloud(accident_id, update_payload)
            # --- END CLOUD LOG ---
            
            return True
    except:
        pass
    return False

def clear_accident(erv_id, accident_id):
    if accident_id not in accident_status:
        return
    
    try:
        vehicles_to_clear = accident_vehicles.get(accident_id, [])
        
        for veh_id in vehicles_to_clear:
            if veh_id in traci.vehicle.getIDList():
                try:
                    traci.vehicle.remove(veh_id)
                    print(f"[CLEAR] {veh_id} removed")
                except:
                    pass
        
        accident_status[accident_id]['cleared'] = True
        print(f"[CLEAR] {accident_id} cleared")

        # --- CLOUD LOG ---
        clear_time = traci.simulation.getTime()
        detection_time = accident_status[accident_id].get('detection_time')
        arrival_time = erv_arrival_times.get(erv_id)
        response_time = None
        if detection_time is not None and arrival_time is not None:
            response_time = arrival_time - detection_time

        update_payload = {
            "clear_time": clear_time,
            "response_time": response_time,
            "status": "cleared"
        }
        update_accident_in_cloud(accident_id, update_payload)
        # --- END CLOUD LOG ---

        # Immediately return ambulance to parking after clearing
        return_ambulance_to_parking(erv_id)
        
    except:
        pass

def return_ambulance_to_parking(erv_id):
    """Immediately teleport ambulance back to its parking position"""
    if erv_id not in traci.vehicle.getIDList():
        return
    
    try:
        # Get parking info
        amb_num = erv_id.replace('ambulance', '')
        parking_route = f"routeAmbulance{amb_num}"
        parking_edges = traci.route.getEdges(parking_route)
        
        if not parking_edges:
            print(f"[RETURN] No parking edges for {erv_id}")
            return
        
        parking_edge = parking_edges[0]
        lane_id = parking_edge + "_0"
        lane_length = traci.lane.getLength(lane_id)
        pos = lane_length / 2.0
        
        # DIRECTLY TELEPORT ambulance to parking position
        traci.vehicle.moveTo(erv_id, lane_id, pos)
        
        # Stop it permanently
        traci.vehicle.setSpeed(erv_id, 0)
        traci.vehicle.setMaxSpeed(erv_id, 0)
        traci.vehicle.setStop(erv_id, edgeID=parking_edge, pos=pos, duration=int(1e9))
        
        # Reset to idle state immediately
        erv_status[erv_id] = 'idle'
        erv_target_edge[erv_id] = None
        erv_assignments[erv_id] = None
        erv_destinations[erv_id] = None
        erv_optimal_route[erv_id] = None
        erv_arrival_times[erv_id] = None
        
        print(f"[RETURN] {erv_id} teleported back to parking - IDLE")
        
    except Exception as e:
        print(f"[RETURN ERROR] {erv_id}: {e}")

def reset_erv_after_service(erv_id):
    if erv_id not in traci.vehicle.getIDList():
        return
    
    try:
        amb_num = erv_id.replace('ambulance', '')
        parking_route = f"routeAmbulance{amb_num}"
        
        parking_edges = traci.route.getEdges(parking_route)
        if not parking_edges:
            return
            
        parking_edge = parking_edges[0]
        
        try:
            current_edge = traci.vehicle.getRoadID(erv_id)
        except:
            return
        
        try:
            traci.vehicle.setStop(erv_id, edgeID="", pos=0, duration=0, flags=0)
        except:
            pass
        
        if current_edge != parking_edge:
            try:
                route_result = traci.simulation.findRoute(current_edge, parking_edge)
                
                if route_result and route_result.edges and len(route_result.edges) > 0:
                    traci.vehicle.setRoute(erv_id, list(route_result.edges))
                else:
                    traci.vehicle.setRoute(erv_id, [current_edge, parking_edge])
                
                traci.vehicle.setMaxSpeed(erv_id, 15.0)
                traci.vehicle.setSpeed(erv_id, -1)
                
            except Exception as e:
                print(f"[ERV] Reset route error: {e}")
                return
        
        try:
            lane_id = parking_edge + "_0"
            lane_length = traci.lane.getLength(lane_id)
            pos = lane_length / 2.0
            traci.vehicle.setStop(erv_id, edgeID=parking_edge, pos=pos, duration=int(1e9))
        except:
            pass
        
        erv_status[erv_id] = 'returning'
        erv_assignments[erv_id] = None
        erv_destinations[erv_id] = None
        erv_target_edge[erv_id] = parking_edge
        erv_optimal_route[erv_id] = None
        erv_arrival_times[erv_id] = None
        
        print(f"[ERV] {erv_id} returning to parking")
        
    except Exception as e:
        print(f"[ERV] Reset error: {e}")

def manage_erv_operations(ambulance_keys):
    for erv_id in ambulance_keys:
        initialize_erv_state(erv_id)
        
        if erv_status[erv_id] == 'assigned':
            accident_id = erv_assignments[erv_id]
            if accident_id in accident_status:
                dispatch_erv(erv_id, erv_target_edge[erv_id], erv_destinations[erv_id])
        
        elif erv_status[erv_id] == 'en_route':
            accident_id = erv_assignments[erv_id]
            if accident_id in accident_status:
                check_erv_arrival(erv_id, accident_id)
        
        elif erv_status[erv_id] == 'arrived':
            accident_id = erv_assignments[erv_id]
            if accident_id in accident_status and not accident_status[accident_id]['cleared']:
                clear_accident(erv_id, accident_id)
                cen.mark_accident_cleared(accident_id)
                # Ambulance is now back at parking and idle (teleported)

# -------------------- START SUMO --------------------
traci.start([sumoBinary, "-c", sumoConfig,
             "--collision.action", "none",
             "--collision.check-junctions", "true",
             "--ignore-route-errors", "true"], port=8813)

initialize_edge_nodes()
cen = CENBroadcast(interval=10, edge_nodes=edge_nodes)

graph = {
    "A": [("B", 100), ("D", 100)],
    "B": [("A", 100), ("C", 100), ("E", 100)],
    "C": [("B", 100), ("F", 100)],
    "D": [("A", 100), ("E", 100), ("G", 100)],
    "E": [("B", 100), ("D", 100), ("F", 100), ("H", 100)],
    "F": [("C", 100), ("E", 100), ("I", 100)],
    "G": [("D", 100), ("H", 100)],
    "H": [("G", 100), ("E", 100), ("I", 100)],
    "I": [("F", 100), ("H", 100)]
}

# -------------------- SPAWN AMBULANCES --------------------
ambulance_parking_routes = {
    "ambulance0": "routeAmbulance0",
    "ambulance1": "routeAmbulance1",
    "ambulance2": "routeAmbulance2"
}

for vid, route in ambulance_parking_routes.items():
    try:
        traci.vehicle.add(vid, routeID=route, typeID="ambulance", depart=0)
        traci.vehicle.setSpeed(vid, 0)
        traci.vehicle.setMaxSpeed(vid, 0)
        traci.vehicle.setEmergencyDecel(vid, 1000)
        traci.vehicle.setTau(vid, 0)
        traci.vehicle.setMinGap(vid, 0)
        parking_edge = traci.route.getEdges(route)[0]
        lane_id = parking_edge + "_0"
        lane_length = traci.lane.getLength(lane_id)
        pos = lane_length / 2.0
        traci.vehicle.setStop(vid, edgeID=parking_edge, pos=pos, duration=1e6)
        initialize_erv_state(vid)
    except Exception as e:
        print(f"Failed to spawn {vid}: {e}")

ambulance_readiness = {amb_id: True for amb_id in ambulance_parking_routes.keys()}

# -------------------- SPAWN VEHICLES --------------------
def spawn_vehicle(step):
    global VEHICLE_COUNTER
    vid = f"veh{VEHICLE_COUNTER}"
    VEHICLE_COUNTER += 1
    r = random.choice(routes)
    t = random.choice(list(vtypes.keys()))
    try:
        traci.vehicle.add(vid, routeID=r, typeID=t, depart=step)
        traci.vehicle.setEmergencyDecel(vid, 1000)
        traci.vehicle.setTau(vid, 0)
        traci.vehicle.setMinGap(vid, 0)
        max_speed = float(vtypes[t].attrib.get("maxSpeed", 13.9))
        speed = random.uniform(max_speed*0.3, max_speed)
        traci.vehicle.setSpeed(vid, speed)
        vehicles_dict[vid] = Vehicle(veh_id=vid, destination=traci.vehicle.getRoute(vid)[-1])
    except:
        pass

for _ in range(NUM_INITIAL_VEHICLES):
    spawn_vehicle(step=0)

# -------------------- SIMULATION LOOP --------------------
step = 0
MAX_STEPS = 500
while step < MAX_STEPS:
    traci.simulationStep()
    step += 1
    time.sleep(0.5)

    if step % SPAWN_INTERVAL == 0:
        for _ in range(random.randint(1, 2)):
            spawn_vehicle(step)

    vehicles = [v for v in traci.vehicle.getIDList() if not v.startswith('ambulance')]
    positions = {vid: get_vehicle_position(vid) for vid in vehicles}

    new_collisions = []
    for vid1, pos1 in positions.items():
        for vid2, pos2 in positions.items():
            if vid1 >= vid2:
                continue
            distance = calculate_distance(pos1, pos2)
            pair = tuple(sorted([vid1, vid2]))
            if distance < COLLISION_DISTANCE and pair not in reported_collisions:
                for v in pair:
                    try:
                        traci.vehicle.setSpeed(v, 0)
                        traci.vehicle.setColor(v, (255,0,0,255))
                        current_edge = traci.vehicle.getRoadID(v)
                        current_pos = traci.vehicle.getLanePosition(v)
                        traci.vehicle.setStop(v, edgeID=current_edge, pos=current_pos, duration=999999)
                    except:
                        pass
                reported_collisions.add(pair)
                new_collisions.append((pair, pos1))
                total_accidents += 1
                print(f"[ACCIDENT {total_accidents}] {pair} at {pos1}")

    for collision_pair, location in new_collisions:
        accident_id = f"ACC_{total_accidents:03d}"
        x, y = location
        sim_time = traci.simulation.getTime()
        
        # Detect accident edge
        accident_edge = detect_accident(collision_pair[0], collision_pair[1], x, y)
        
        # --- CLOUD LOG ---
        initial_payload = {
            "accident_id": accident_id,
            "location_x": x,
            "location_y": y,
            "accident_edge": accident_edge,
            "vehicles_involved": list(collision_pair),
            "detection_time": sim_time,
            "status": "detected"
        }
        log_new_accident_to_cloud(initial_payload)
        # --- END CLOUD LOG ---

        # Register with CEN
        cen.register(accident_id, (x, y), sim_time, list(collision_pair), accident_edge=accident_edge)
        
        # Collect ambulance positions
        amb_positions = {}
        for amb in ambulance_readiness.keys():
            try:
                amb_positions[amb] = traci.vehicle.getPosition(amb)
            except:
                continue
        
        # GA selection with route optimization
        best_ambulance, optimal_route = select_best_ambulance(x, y, amb_positions, accident_edge)
        
        # Assign and dispatch with optimal route
        if best_ambulance and erv_status.get(best_ambulance, 'idle') == 'idle':
            assign_erv_to_accident(best_ambulance, accident_id, x, y, accident_edge, collision_pair, optimal_route)
        else:
            available_erv = get_available_erv(ambulance_readiness.keys())
            if available_erv:
                assign_erv_to_accident(available_erv, accident_id, x, y, accident_edge, collision_pair, optimal_route)
            else:
                print(f"[WARNING] No available ERV for {accident_id}")

    manage_erv_operations(ambulance_readiness.keys())

    for vid, vehicle in vehicles_dict.items():
        if vid not in traci.vehicle.getIDList():
            continue
        vehicle.listen_and_reroute(cen, {k:v['position'] for k,v in edge_nodes.items()}, graph, comm_range=V2V_COMMUNICATION_RANGE)

    cen.broadcast(traci.simulation.getTime(), vehicles_dict=vehicles_dict, graph=graph, comm_range=V2V_COMMUNICATION_RANGE)

traci.close()
