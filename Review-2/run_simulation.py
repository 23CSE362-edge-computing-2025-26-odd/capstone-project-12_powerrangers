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

# -------------------- SUMO PATH SETUP --------------------
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)
from best_erv import select_best_ambulance
# -------------------- SUMO SETUP --------------------
sumoBinary = sumolib.checkBinary("sumo-gui")
sumoConfig = "simulation.sumocfg"
vehicles_dict = {}
V2V_COMMUNICATION_RANGE = 200.0
MAX_HOP_COUNT = 5
COLLISION_DISTANCE = 7.5

# Edge node positions (fixed infrastructure nodes)
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

# -------------------- ERV MANAGEMENT STATE --------------------
erv_assignments = {}  # Maps ERV ID to accident ID
erv_status = {}  # Maps ERV ID to status ('idle', 'assigned', 'en_route', 'arrived')
erv_destinations = {}  # Maps ERV ID to (x, y) destination
erv_target_edge = {}  # Maps ERV ID to target edge ID
accident_status = {}  # Maps accident ID to {'location': (x,y), 'erv': erv_id, 'cleared': bool, 'vehicles': []}
erv_arrival_times = {}  # Maps ERV ID to arrival time at accident
accident_vehicles = {}  # Maps accident ID to list of vehicle IDs involved

# -------------------- PARSE ROUTES AND VEHICLE TYPES --------------------
rou_file = "vehicles.rou.xml"
tree = ET.parse(rou_file)
root = tree.getroot()
routes = [r.attrib['id'] for r in root.findall("route") if not r.attrib['id'].startswith('routeAmbulance')]
vtypes = {v.attrib['id']: v for v in root.findall("vType") if v.attrib['id'] != 'ambulance'}

print("Loaded routes:", len(routes))
print("Loaded vehicle types:", list(vtypes.keys()))

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
    print("EDGE NODE INITIALIZATION")
    print("=" * 50)
    for node_id, position in EDGE_NODE_POSITIONS.items():
        edge_nodes[node_id] = {
            'position': position,
            'connected_vehicles': set(),
            'message_cache': set(),
            'total_messages_received': 0,
            'unique_accidents_reported': set(),
            'accident_alerts_received': []
        }
        print(f"Edge Node: {node_id} - Position: {position}")
    print("=" * 50)

# -------------------- HELPER FUNCTIONS --------------------
# -------------------- HELPER FUNCTIONS WITH LOGGING --------------------
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
    if vehicles_in_range:
        print(f"    Vehicles in range of {exclude_vehicle}: {[v[0] for v in vehicles_in_range]}")
    return vehicles_in_range

def find_edge_nodes_in_range(position):
    edge_nodes_in_range = []
    for edge_id, edge_info in edge_nodes.items():
        distance = calculate_distance(position, edge_info['position'])
        if distance <= V2V_COMMUNICATION_RANGE:
            edge_nodes_in_range.append((edge_id, distance))
    edge_nodes_in_range.sort(key=lambda x: x[1])
    if edge_nodes_in_range:
        print(f"    Edge nodes in range at pos {position}: {[e[0] for e in edge_nodes_in_range]}")
    return edge_nodes_in_range

def propagate_v2v_message(message, current_vehicle_id, current_position):
    print(f"    [V2V] Vehicle {current_vehicle_id} propagating message {message.message_id}, hop {message.hop_count}")
    if message.hop_count >= MAX_HOP_COUNT:
        print(f"        [V2V] Message {message.message_id} reached max hops ({MAX_HOP_COUNT})")
        return False

    # Check edge nodes in range
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
            print(f"        [CEN] Edge node {edge_id} received message {message.message_id} about accident {message.payload.get('accident_id')}")
            return True

    # Propagate to vehicles in range
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
        print(f"        [V2V] Vehicle {current_vehicle_id} sending message {message.message_id} to vehicle {vehicle_id}")
        success = propagate_v2v_message(next_message, vehicle_id, vehicle_pos)
        if success:
            propagation_successful = True
            break
    return propagation_successful

def broadcast_emergency_alert(source_vehicle_id, accident_location, collision_pair, accident_id):
    global successful_notifications
    print(f"[ACCIDENT] Accident {accident_id} occurred between vehicles {collision_pair} at location {accident_location}")
    source_position = get_vehicle_position(source_vehicle_id)
    if not source_position:
        print(f"    [WARNING] Source vehicle {source_vehicle_id} position not found")
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
        print(f"    [SUCCESS] Emergency alert {message_id} successfully propagated to edge nodes")
    else:
        print(f"    [FAILURE] Emergency alert {message_id} propagation failed")
    return success

def detect_accident(veh1, veh2, x, y):
    """Detects accident and returns the accident edge."""
    try:
        return traci.vehicle.getRoadID(veh1)
    except Exception:
        return None

# -------------------- ERV-SPECIFIC FUNCTIONS --------------------
def initialize_erv_state(erv_id):
    """Initialize state for an ERV."""
    if erv_id not in erv_status:
        erv_status[erv_id] = 'idle'
        erv_assignments[erv_id] = None
        erv_destinations[erv_id] = None
        erv_target_edge[erv_id] = None
        erv_arrival_times[erv_id] = None

def get_available_erv(ambulance_readiness_keys):
    """Find an available (idle) ERV."""
    for erv_id in ambulance_readiness_keys:
        if erv_status.get(erv_id, 'idle') == 'idle':
            return erv_id
    return None

def assign_erv_to_accident(erv_id, accident_id, accident_x, accident_y, accident_edge, collision_pair):
    """Assign an ERV to an accident."""
    erv_assignments[erv_id] = accident_id
    erv_status[erv_id] = 'assigned'
    erv_destinations[erv_id] = (accident_x, accident_y)
    erv_target_edge[erv_id] = accident_edge
    accident_status[accident_id] = {
        'location': (accident_x, accident_y),
        'erv': erv_id,
        'cleared': False,
        'vehicles': list(collision_pair)
    }
    accident_vehicles[accident_id] = list(collision_pair)
    print(f"[ERV ASSIGNMENT] {erv_id} assigned to accident {accident_id}")
    print(f"{erv_id} dispatched to {accident_id}")

def dispatch_erv(erv_id, accident_edge, accident_location):
    """Dispatch an ERV to move toward the accident."""
    try:
        if erv_id not in traci.vehicle.getIDList():
            print(f"[ERV] {erv_id} not found in simulation")
            return False
        
        # Clear parking and enable movement
        try:
            traci.vehicle.setStop(erv_id, edgeID="", pos=0, duration=0, flags=0)
        except:
            pass
        
        try:
            traci.vehicle.setMaxSpeed(erv_id, 30.0)
            traci.vehicle.setSpeed(erv_id, -1)
        except:
            pass
        
        # Set route to accident edge
        if accident_edge:
            try:
                current_edge = traci.vehicle.getRoadID(erv_id)
                if current_edge and current_edge != accident_edge:
                    # Compute route from current edge to accident edge
                    route_edges = traci.simulation.findRoute(current_edge, accident_edge)
                    if route_edges and route_edges.edges:
                        traci.vehicle.setRoute(erv_id, list(route_edges.edges))
                        print(f"[ERV DISPATCH] {erv_id} routed to accident edge {accident_edge}")
                        print(f"{erv_id} en route to {erv_assignments[erv_id]}...")
                    else:
                        # Fallback: set direct route
                        traci.vehicle.setRoute(erv_id, [accident_edge])
                        print(f"[ERV DISPATCH] {erv_id} set direct route to {accident_edge}")
                        print(f"{erv_id} en route to {erv_assignments[erv_id]}...")
            except Exception as e:
                print(f"[ERV DISPATCH] Failed to route {erv_id}: {e}")
                return False
        
        erv_status[erv_id] = 'en_route'
        return True
    except Exception as e:
        print(f"[ERV DISPATCH ERROR] {erv_id}: {e}")
        return False

def check_erv_arrival(erv_id, accident_id):
    """Check if an ERV has reached the accident location."""
    if erv_id not in traci.vehicle.getIDList():
        return False
    
    if accident_id not in accident_status:
        return False
    
    try:
        erv_pos = traci.vehicle.getPosition(erv_id)
        acc_x, acc_y = accident_status[accident_id]['location']
        distance = calculate_distance(erv_pos, (acc_x, acc_y))
        
        # Check if ERV is on target edge or very close to accident
        current_edge = traci.vehicle.getRoadID(erv_id)
        target_edge = erv_target_edge.get(erv_id)
        
        # ERV has arrived when within 20 meters OR on the accident edge
        if distance <= 20.0 or (target_edge and current_edge == target_edge):
            erv_status[erv_id] = 'arrived'
            erv_arrival_times[erv_id] = traci.simulation.getTime()
            return True
    except Exception as e:
        print(f"[ERV ARRIVAL CHECK ERROR] {erv_id}: {e}")
    
    return False

def clear_accident(erv_id, accident_id):
    """Clear an accident: remove accident vehicles and allow traffic to resume."""
    try:
        if accident_id not in accident_status:
            print(f"[ERROR] Accident {accident_id} not found")
            return False
        
        print(f"🚨 {erv_id} reached accident {accident_id}, clearing road...")
        
        # Remove accident vehicles
        vehicles_to_remove = accident_status[accident_id]['vehicles']
        for veh in vehicles_to_remove:
            if veh in traci.vehicle.getIDList():
                try:
                    traci.vehicle.remove(veh)
                    print(f"    [CLEANUP] Vehicle {veh} removed from accident site")
                except Exception as e:
                    print(f"    [CLEANUP WARNING] Could not remove {veh}: {e}")
        
        accident_status[accident_id]['cleared'] = True
        print(f"✅ Road cleared for traffic. {erv_id} ready for next emergency.")
        
        return True
    except Exception as e:
        print(f"[ACCIDENT CLEARING ERROR] {erv_id}, {accident_id}: {e}")
        return False

def reset_erv_after_service(erv_id):
    """Reset an ERV to idle state and return to parking."""
    try:
        if erv_id in traci.vehicle.getIDList():
            # Get the original parking route
            amb_num = erv_id.replace('ambulance', '')
            parking_route = f"routeAmbulance{amb_num}"
            
            try:
                # Get parking edge
                parking_edge = traci.route.getEdges(parking_route)[0]
                current_edge = traci.vehicle.getRoadID(erv_id)
                
                # Compute route back to parking
                if current_edge and current_edge != parking_edge:
                    route_to_parking = traci.simulation.findRoute(current_edge, parking_edge)
                    if route_to_parking and route_to_parking.edges:
                        traci.vehicle.setRoute(erv_id, list(route_to_parking.edges))
                        print(f"[ERV RESET] {erv_id} returning to parking at {parking_edge}")
                    else:
                        # Fallback: direct route
                        traci.vehicle.setRoute(erv_id, [parking_edge])
                        print(f"[ERV RESET] {erv_id} returning to parking (direct route)")
                
                # Set moderate speed for return trip
                traci.vehicle.setMaxSpeed(erv_id, 15.0)
                traci.vehicle.setSpeed(erv_id, -1)
                
                # Set stop at parking location once arrived
                lane_id = parking_edge + "_0"
                lane_length = traci.lane.getLength(lane_id)
                pos = lane_length / 2.0
                traci.vehicle.setStop(erv_id, edgeID=parking_edge, pos=pos, duration=1e6)
                
            except Exception as e:
                print(f"[ERV RESET] {erv_id} parking reset attempted: {e}")
        
        erv_status[erv_id] = 'idle'
        erv_assignments[erv_id] = None
        erv_destinations[erv_id] = None
        erv_target_edge[erv_id] = None
        erv_arrival_times[erv_id] = None
    except Exception as e:
        print(f"[ERV RESET ERROR] {erv_id}: {e}")

def manage_erv_operations(ambulance_readiness_keys):
    """Manage ERV assignments, dispatch, arrival, and cleanup."""
    for erv_id in ambulance_readiness_keys:
        initialize_erv_state(erv_id)
        
        if erv_status[erv_id] == 'assigned':
            accident_id = erv_assignments[erv_id]
            if accident_id in accident_status:
                dispatch_erv(erv_id, erv_target_edge[erv_id], erv_destinations[erv_id])
        
        elif erv_status[erv_id] == 'en_route':
            accident_id = erv_assignments[erv_id]
            if accident_id in accident_status:
                if check_erv_arrival(erv_id, accident_id):
                    print(f"[ERV ARRIVAL] {erv_id} has arrived at accident {accident_id}")
        
        elif erv_status[erv_id] == 'arrived':
            accident_id = erv_assignments[erv_id]
            if accident_id in accident_status and not accident_status[accident_id]['cleared']:
                clear_accident(erv_id, accident_id)
                # Wait a moment before resetting
                if traci.simulation.getTime() - erv_arrival_times.get(erv_id, 0) > 2.0:
                    reset_erv_after_service(erv_id)

# -------------------- START SUMO --------------------
traci.start([sumoBinary, "-c", sumoConfig,
             "--collision.action", "none",
             "--collision.check-junctions", "true",
             "--ignore-route-errors", "true"], port=8813)

# -------------------- INITIALIZE EDGE NODES --------------------
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
        print(f"Failed to spawn ambulance {vid}: {e}")


# -------------------- Initialize ambulance readiness dictionary --------------------
ambulance_readiness = {amb_id: True for amb_id in ambulance_parking_routes.keys()}

# -------------------- SPAWN VEHICLES FUNCTION --------------------
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
    except Exception as e:
        print(f"Failed to spawn vehicle {vid}: {e}")

# -------------------- SPAWN INITIAL VEHICLES --------------------
for _ in range(NUM_INITIAL_VEHICLES):
    spawn_vehicle(step=0)

# -------------------- SIMULATION LOOP --------------------
step = 0
MAX_STEPS = 500
while step < MAX_STEPS:
    traci.simulationStep()
    step += 1
    time.sleep(0.5)

    # Spawn new vehicles
    if step % SPAWN_INTERVAL == 0:
        for _ in range(random.randint(1, 2)):
            spawn_vehicle(step)

    # Track vehicles
    vehicles = [v for v in traci.vehicle.getIDList() if not v.startswith('ambulance')]
    positions = {vid: get_vehicle_position(vid) for vid in vehicles}

    # Detect new collisions
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
                print(f"[ACCIDENT] Vehicles involved: {pair} at position {pos1} (Total accidents: {total_accidents})")

    # Register accidents in CEN and dispatch ambulances
    for collision_pair, location in new_collisions:
        accident_id = f"ACC_{total_accidents:03d}"
        x, y = location
        sim_time = traci.simulation.getTime()
        source_vehicle = collision_pair[0]
        cen.register(accident_id, (x, y), sim_time, source_vehicle)
        
        # Detect accident edge and select best ambulance
        accident_edge = detect_accident(collision_pair[0], collision_pair[1], x, y)
        
        # Collect ambulance positions
        amb_positions = {}
        for amb in ambulance_readiness.keys():
            try:
                amb_positions[amb] = traci.vehicle.getPosition(amb)
            except Exception:
                continue
        
        # GA call to select best ambulance
        best_ambulance = select_best_ambulance(x, y, amb_positions, accident_edge)
        
        # Assign and dispatch ERV - only if it's available
        if best_ambulance and erv_status.get(best_ambulance, 'idle') == 'idle':
            assign_erv_to_accident(best_ambulance, accident_id, x, y, accident_edge, collision_pair)
            print(f"[AMBULANCE] Dispatching {best_ambulance} to accident {accident_id} at ({x}, {y})")
        else:
            # Find another available ERV
            available_erv = get_available_erv(ambulance_readiness.keys())
            if available_erv:
                assign_erv_to_accident(available_erv, accident_id, x, y, accident_edge, collision_pair)
                print(f"[AMBULANCE] Dispatching {available_erv} to accident {accident_id} at ({x}, {y})")
            else:
                print(f"[WARNING] No available ERV for accident {accident_id}")

    # Manage ERV operations (dispatch, arrival detection, cleanup)
    manage_erv_operations(ambulance_readiness.keys())

    # Vehicles listen to CEN broadcasts
    for vid, vehicle in vehicles_dict.items():
        if vid not in traci.vehicle.getIDList():
            continue
        vehicle.listen_and_reroute(cen, {k:v['position'] for k,v in edge_nodes.items()}, graph, comm_range=V2V_COMMUNICATION_RANGE)

    # Periodic CEN broadcast
    cen.broadcast(traci.simulation.getTime(), vehicles_dict=vehicles_dict, graph=graph, comm_range=V2V_COMMUNICATION_RANGE)

traci.close()
