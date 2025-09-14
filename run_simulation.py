import sys
import os
import time
import random
import xml.etree.ElementTree as ET
import math
import uuid
from collections import defaultdict
from cen_broadcast import CENBroadcast
cen = CENBroadcast(interval=300) 

# -------------------- SUMO PATH SETUP --------------------
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)

from Vehicle_Rerouting import blocked_edges, stopped_vehicles, reroute_vehicles, handle_accident, clear_blocked_edges, remove_stopped_vehicle
import traci
import sumolib

# ----------------------------
# SUMO setup
sumoBinary = sumolib.checkBinary("sumo-gui")
sumoConfig = "simulation.sumocfg"

# ----------------------------
# V2V Communication Configuration
V2V_COMMUNICATION_RANGE = 200.0  # meters
MAX_HOP_COUNT = 5
COLLISION_DISTANCE = 7.5  # meters threshold

# Edge node positions (fixed infrastructure nodes)
EDGE_NODE_POSITIONS = {
    "EdgeNode_A": (30, 20),      # Near intersection A
    "EdgeNode_D": (0, 100),    # Near intersection D
    "EdgeNode_C": (200, 0),    # Near intersection C
    "EdgeNode_I": (200, 200)   # Near intersection I
}

# ----------------------------
# Global State Management
edge_nodes = {}
broadcasted_accidents = set()
message_history = {}
hop_count_stats = defaultdict(int)
reported_collisions = set()
total_accidents = 0
successful_notifications = 0

# ----------------------------
# Parse routes and vehicle types from vehicles.rou.xml
rou_file = "vehicles.rou.xml"
tree = ET.parse(rou_file)
root = tree.getroot()

routes = [r.attrib['id'] for r in root.findall("route") 
          if not r.attrib['id'].startswith('routeAmbulance')]
vtypes = {v.attrib['id']: v for v in root.findall("vType") if v.attrib['id'] != 'ambulance'}

print("Loaded routes:", len(routes))
print("Loaded vehicle types:", list(vtypes.keys()))

# ----------------------------
NUM_INITIAL_VEHICLES = 4
SPAWN_INTERVAL = 20
VEHICLE_COUNTER = 0

# ----------------------------
class V2VMessage:
    """Represents a V2V communication message"""
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

def initialize_edge_nodes():
    """Initialize edge nodes in the communication network"""
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
        print(f"Edge Node: {node_id} - Position: ({position[0]}, {position[1]})")
    print("=" * 50)

def get_vehicle_position(vehicle_id):
    """Get vehicle position safely"""
    try:
        if vehicle_id in traci.vehicle.getIDList():
            return traci.vehicle.getPosition(vehicle_id)
    except:
        pass
    return None

def calculate_distance(pos1, pos2):
    """Calculate Euclidean distance between two positions"""
    if pos1 and pos2:
        return math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
    return float('inf')

def find_vehicles_in_range(source_position, exclude_vehicle=None):
    """Find all vehicles within V2V communication range (excluding ambulances)"""
    vehicles_in_range = []
    
    for vehicle_id in traci.vehicle.getIDList():
        if vehicle_id == exclude_vehicle:
            continue
        if vehicle_id.startswith('ambulance'):  # Skip ambulances
            continue
        if vehicle_id in stopped_vehicles:  # Skip stopped vehicles
            continue
            
        vehicle_pos = get_vehicle_position(vehicle_id)
        if vehicle_pos:
            distance = calculate_distance(source_position, vehicle_pos)
            if distance <= V2V_COMMUNICATION_RANGE:
                vehicles_in_range.append((vehicle_id, distance, vehicle_pos))
    
    return sorted(vehicles_in_range, key=lambda x: x[1])

def find_edge_nodes_in_range(position):
    """Find edge nodes within communication range"""
    edge_nodes_in_range = []
    
    for edge_id, edge_info in edge_nodes.items():
        edge_pos = edge_info['position']
        distance = calculate_distance(position, edge_pos)
        if distance <= V2V_COMMUNICATION_RANGE:
            edge_nodes_in_range.append((edge_id, distance))
    
    return sorted(edge_nodes_in_range, key=lambda x: x[1])

def propagate_v2v_message(message, current_vehicle_id, current_position):
    """Implement multi-hop V2V message propagation to reach edge nodes"""
    
    print(f"    Propagating from {current_vehicle_id} at hop {message.hop_count}")
    
    if message.hop_count >= MAX_HOP_COUNT:
        print(f"    Message reached maximum hops ({MAX_HOP_COUNT})")
        return False
    
    # Priority: Check if message can reach an edge node directly
    edge_nodes_in_range = find_edge_nodes_in_range(current_position)
    print(f"    Found {len(edge_nodes_in_range)} edge nodes in range")
    
    for edge_id, distance in edge_nodes_in_range:
        if message.message_id not in edge_nodes[edge_id]['message_cache']:
            edge_nodes[edge_id]['message_cache'].add(message.message_id)
            edge_nodes[edge_id]['total_messages_received'] += 1
            
            if message.message_type == "EMERGENCY":
                edge_nodes[edge_id]['unique_accidents_reported'].add(message.payload.get('accident_id'))
                # Store detailed accident alert info
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
            
            print(f"    SUCCESS: Edge node {edge_id} contacted")
            print(f"    Distance to edge node: {distance:.1f}m")
            print(f"    Final hop count: {message.hop_count}")
            print(f"    Propagation path: {' -> '.join(message.propagation_path)}")
            
            hop_count_stats[message.hop_count] += 1
            return True
    
    # Continue propagation to nearby vehicles if no direct edge connection
    vehicles_in_range = find_vehicles_in_range(current_position, exclude_vehicle=current_vehicle_id)
    print(f"    Found {len(vehicles_in_range)} vehicles in range for next hop")
    
    propagation_successful = False
    for vehicle_id, distance, vehicle_pos in vehicles_in_range:
        
        if vehicle_id in message.propagation_path:
            print(f"    Skipping {vehicle_id} (already in path)")
            continue  # Avoid loops
        
        print(f"    Attempting hop {message.hop_count + 1}: {current_vehicle_id} -> {vehicle_id} (Distance: {distance:.1f}m)")
        
        # Create a new message instance for this hop
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
            break  # Stop after first successful path
    
    if not propagation_successful:
        print(f"    Failed to propagate further from {current_vehicle_id}")
    
    return propagation_successful

def broadcast_emergency_alert(source_vehicle_id, accident_location, collision_pair, accident_id):
    """Broadcast emergency alert using multi-hop V2V communication"""
    global successful_notifications
    
    source_position = get_vehicle_position(source_vehicle_id)
    if not source_position:
        return False
    
    # Create emergency message
    message_id = f"EMERGENCY_{uuid.uuid4().hex[:8]}"
    
    emergency_payload = {
        'accident_id': accident_id,
        'vehicles_involved': collision_pair,
        'location': accident_location,
        'severity': 'HIGH',
        'timestamp': traci.simulation.getTime()
    }
    
    message = V2VMessage(
        message_id=message_id,
        source_id=source_vehicle_id,
        message_type="EMERGENCY",
        payload=emergency_payload,
        origin_location=accident_location
    )
    
    print(f"  V2V Emergency Broadcast:")
    print(f"    Source Vehicle: {source_vehicle_id}")
    print(f"    Message ID: {message_id}")
    print(f"    Starting V2V propagation...")
    
    message_history[message_id] = [message]
    
    success = propagate_v2v_message(message, source_vehicle_id, source_position)
    
    if success:
        successful_notifications += 1
        print(f"    Result: Edge node successfully notified")
        return True
    else:
        print(f"    Result: Failed to reach any edge node")
        return False

# ----------------------------
# Start SUMO
traci.start([
    sumoBinary, "-c", sumoConfig,
    "--collision.action", "none",
    "--collision.check-junctions", "true",
    "--ignore-route-errors", "true"
], port=8813)

# ----------------------------
# Initialize edge nodes
initialize_edge_nodes()

# ----------------------------
# Spawn ambulances at fixed parking edges (stationary)
ambulance_parking_routes = {
    "ambulance0": "routeAmbulance0",
    "ambulance1": "routeAmbulance1",
    "ambulance2": "routeAmbulance2"
}

print("AMBULANCE DEPLOYMENT")
print("=" * 30)
for vid, route in ambulance_parking_routes.items():
    try:
        # Add vehicle on parking route
        traci.vehicle.add(vid, routeID=route, typeID="ambulance", depart=0)

        # Keep ambulance completely stationary
        traci.vehicle.setSpeed(vid, 0)
        traci.vehicle.setMaxSpeed(vid, 0)
        traci.vehicle.setEmergencyDecel(vid, 1000)
        traci.vehicle.setTau(vid, 0)
        traci.vehicle.setMinGap(vid, 0)

        # Stop vehicle permanently at the start of the parking edge
        parking_edge = traci.route.getEdges(route)[0]
        lane_id = parking_edge + "_0"
        lane_length = traci.lane.getLength(lane_id)
        pos = lane_length / 2.0

        traci.vehicle.setStop(vid, edgeID=parking_edge, pos=pos, duration=1e6)
        stopped_vehicles.add(vid)
        print(f"Ambulance {vid}: Parked and inactive")
    except Exception as e:
        print(f"Failed to spawn ambulance {vid}: {e}")
print("=" * 30)

# ----------------------------
# Function to spawn a normal vehicle
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

        # Random speed
        max_speed = float(vtypes[t].attrib.get("maxSpeed", 13.9))
        speed = random.uniform(max_speed * 0.3, max_speed)
        traci.vehicle.setSpeed(vid, speed)

        print(f"Spawned {vid} on {r} as {t} at speed {speed:.1f} m/s at step {step}")
        return True
    except Exception as e:
        print(f"Failed to spawn vehicle {vid}: {e}")
        return False


# ----------------------------
# Spawn initial vehicles
for _ in range(NUM_INITIAL_VEHICLES):
    spawn_vehicle(step=0)

# ----------------------------
# Run simulation
step = 0
MAX_STEPS = 500
while step < MAX_STEPS:
    traci.simulationStep()
    step += 1
    time.sleep(0.5)

    # Spawn new vehicles periodically
    if step % SPAWN_INTERVAL == 0:
        for _ in range(random.randint(1, 2)):
            spawn_vehicle(step)

    # Get positions of all non-ambulance vehicles
    vehicles = [v for v in traci.vehicle.getIDList() if not v.startswith('ambulance')]
    positions = {}
    
    for vid in vehicles:
        try:
            positions[vid] = traci.vehicle.getPosition(vid)
        except:
            continue

    # Check collisions
    new_collisions = []
    for vid1, pos1 in positions.items():
        for vid2, pos2 in positions.items():
            if vid1 >= vid2:
                continue
            distance = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
            pair = tuple(sorted([vid1, vid2]))
            
            if distance < COLLISION_DISTANCE and pair not in reported_collisions:
                # Stop involved vehicles permanently
                for v in pair:
                    try:
                        traci.vehicle.setSpeed(v, 0)
                        traci.vehicle.setColor(v, (255, 0, 0, 255))  # Red for accident
                        current_edge = traci.vehicle.getRoadID(v)
                        current_pos = traci.vehicle.getLanePosition(v)
                        traci.vehicle.setStop(v, edgeID=current_edge, pos=current_pos, duration=999999)
                        stopped_vehicles.add(v)
                    except:
                        pass

                reported_collisions.add(pair)
                new_collisions.append((pair, pos1))
                total_accidents += 1

    # Handle new accidents with V2V communication
    for collision_pair, location in new_collisions:
        accident_id = f"ACC_{total_accidents:03d}"
        
        x, y = location
        sim_time = traci.simulation.getTime()
        
        print(f"\n⚠️ ACCIDENT DETECTED")
        print("=" * 25)
        print(f"Accident ID: {accident_id}")
        print(f"Time: {sim_time:.1f} seconds")
        print(f"Location: ({x:.2f}, {y:.2f})")
        print(f"Vehicles involved: {collision_pair[0]}, {collision_pair[1]}")
        
        # Broadcast emergency alert using V2V
        source_vehicle = collision_pair[0]  # First vehicle reports the accident
        success = broadcast_emergency_alert(source_vehicle, location, collision_pair, accident_id)
        
        print(f"V2V Notification Status: {'SUCCESS' if success else 'FAILED'}")
        print("=" * 25)

        # Handle the accident - call the function with correct parameters
        rerouted_count = handle_accident(traci, collision_pair[0], collision_pair[1])
        print(f"Rerouted {rerouted_count} vehicles due to accident")
        


        cen.register(accident_id, (x, y), sim_time)

    # Progress update every 50 steps
    if step % 50 == 0:
        current_vehicles = len([v for v in traci.vehicle.getIDList() if not v.startswith('ambulance')])
        active_vehicles = current_vehicles - len([v for v in stopped_vehicles if not v.startswith('ambulance')])
        sim_time = traci.simulation.getTime()
        
        print(f"\n--- STATUS UPDATE - Step {step} (Time: {sim_time:.0f}s) ---")
        print(f"Active vehicles: {active_vehicles}, Stopped vehicles: {current_vehicles - active_vehicles}")
        print(f"Total accidents: {total_accidents}, Edge notifications: {successful_notifications}")
        print(f"Currently blocked edges: {blocked_edges}")

    sim_time = traci.simulation.getTime()
    cen.broadcast(sim_time)
# ----------------------------
# Final Statistics
print("\n" + "=" * 60)
print("FINAL V2V COMMUNICATION REPORT")
print("=" * 60)
print(f"Simulation duration: {traci.simulation.getTime():.0f} seconds")
print(f"Total accidents detected: {total_accidents}")
print(f"Successful edge node notifications: {successful_notifications}")
if total_accidents > 0:
    print(f"Overall notification success rate: {100*successful_notifications/total_accidents:.1f}%")

print(f"\nHop count distribution:")
if hop_count_stats:
    for hops in sorted(hop_count_stats.keys()):
        print(f"  {hops} hops: {hop_count_stats[hops]} messages")
else:
    print("  No successful V2V communications")

print(f"\nDetailed edge node report:")
for edge_id, edge_info in edge_nodes.items():
    print(f"  {edge_id} at {edge_info['position']}:")
    print(f"    Total messages received: {edge_info['total_messages_received']}")
    print(f"    Unique accidents reported: {len(edge_info['unique_accidents_reported'])}")
    
    if edge_info['accident_alerts_received']:
        print(f"    Accident details:")
        for alert in edge_info['accident_alerts_received']:
            loc = alert['location']
            print(f"      {alert['accident_id']}: Location ({loc[0]:.1f}, {loc[1]:.1f}), "
                  f"Hops: {alert['hop_count']}, Time: {alert['received_at']:.1f}s")

print("=" * 60)

# ----------------------------
traci.close()