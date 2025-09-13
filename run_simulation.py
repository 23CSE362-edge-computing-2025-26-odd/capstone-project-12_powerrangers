import sys
import os
import time
import random
import math
import xml.etree.ElementTree as ET
from collections import deque, defaultdict
import uuid

# -------------------- SUMO PATH SETUP --------------------
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)

import traci
import sumolib

# Mock ns-3 classes for simulation without actual ns-3 installation
class MockNS3:
    class NodeContainer:
        def __init__(self): self.nodes = []
        def Create(self, n): self.nodes = [f"node_{i}" for i in range(n)]
        def Get(self, i): return self.nodes[i] if i < len(self.nodes) else None
    
    class MobilityHelper:
        def SetMobilityModel(self, model): pass
        def Install(self, nodes): pass
    
    class WifiHelper:
        def SetStandard(self, std): pass
        def Install(self, phy, mac, nodes): return "mock_devices"
    
    class InternetStackHelper:
        def Install(self, nodes): pass
    
    class Ipv4AddressHelper:
        def __init__(self): self.network_counter = 1
        def SetBase(self, addr, mask): pass
        def Assign(self, devices): pass
        def NewNetwork(self): 
            self.network_counter += 1
            return f"10.1.{self.network_counter}.0"

# Initialize mock ns-3 components
mock_ns3 = MockNS3()

# ----------------------------
# Global State Management
vehicle_nodes = {}
edge_nodes = {}
broadcasted_accidents = set()
message_history = {}  # Track message propagation
hop_count_stats = defaultdict(int)
collision_pairs = set()
stopped_vehicles = set()
ambulance_rerouted = set()
accident_locations = {}

# Configuration - ENHANCED V2V COMMUNICATION RANGE
MAX_STEPS = 1000
COLLISION_DISTANCE = 8.0
V2V_COMMUNICATION_RANGE = 200.0  # INCREASED from 80m to 200m for better coverage
MAX_HOP_COUNT = 5
AMBULANCE_RESPONSE_DISTANCE = 30.0
EMERGENCY_SPEED_BOOST = 1.8

# Edge node positions (fixed infrastructure nodes)
EDGE_NODE_POSITIONS = {
    "EdgeNode_D": (0, 100),
    "EdgeNode_C": (200, 0), 
    "EdgeNode_I": (200, 200)
}

class V2VMessage:
    """Represents a V2V communication message"""
    def __init__(self, message_id, source_id, message_type, payload, origin_location):
        self.message_id = message_id
        self.source_id = source_id
        self.message_type = message_type  # "EMERGENCY", "REROUTE", "ACK"
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
    for node_id, position in EDGE_NODE_POSITIONS.items():
        edge_nodes[node_id] = {
            'position': position,
            'connected_vehicles': set(),
            'message_cache': set(),
            'total_messages_received': 0,
            'unique_accidents_reported': set()
        }
        print(f"Edge Node initialized: {node_id} at position {position}")

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
    """Find all vehicles within V2V communication range - ENHANCED RANGE"""
    vehicles_in_range = []
    
    for vehicle_id in traci.vehicle.getIDList():
        if vehicle_id == exclude_vehicle:
            continue
            
        vehicle_pos = get_vehicle_position(vehicle_id)
        if vehicle_pos:
            distance = calculate_distance(source_position, vehicle_pos)
            if distance <= V2V_COMMUNICATION_RANGE:  # Using enhanced 200m range
                vehicles_in_range.append((vehicle_id, distance, vehicle_pos))
                # Debug: Show enhanced range in action
                if distance > 80:  # Show when using the extended range beyond original 80m
                    print(f"      Extended range V2V: {exclude_vehicle} -> {vehicle_id} ({distance:.1f}m)")
    
    return sorted(vehicles_in_range, key=lambda x: x[1])  # Sort by distance

def find_edge_nodes_in_range(position):
    """Find edge nodes within communication range - ENHANCED RANGE"""
    edge_nodes_in_range = []
    
    for edge_id, edge_info in edge_nodes.items():
        edge_pos = edge_info['position']
        distance = calculate_distance(position, edge_pos)
        if distance <= V2V_COMMUNICATION_RANGE:  # Using enhanced 200m range
            edge_nodes_in_range.append((edge_id, distance))
            # Debug: Show enhanced range in action
            if distance > 80:  # Show when using the extended range beyond original 80m
                print(f"      Extended range to EdgeNode: {edge_id} ({distance:.1f}m)")
    
    return sorted(edge_nodes_in_range, key=lambda x: x[1])

def propagate_v2v_message(message, current_vehicle_id, current_position):
    """Implement multi-hop V2V message propagation with ENHANCED RANGE"""
    
    if message.hop_count >= MAX_HOP_COUNT:
        print(f"    Message {message.message_id} reached max hops ({MAX_HOP_COUNT})")
        return False
    
    # Check if message reached an edge node (with enhanced range)
    edge_nodes_in_range = find_edge_nodes_in_range(current_position)
    for edge_id, distance in edge_nodes_in_range:
        if message.message_id not in edge_nodes[edge_id]['message_cache']:
            edge_nodes[edge_id]['message_cache'].add(message.message_id)
            edge_nodes[edge_id]['total_messages_received'] += 1
            
            if message.message_type == "EMERGENCY":
                edge_nodes[edge_id]['unique_accidents_reported'].add(message.payload.get('accident_id'))
            
            message.reached_edge_node = True
            range_type = "Extended" if distance > 80 else "Standard"
            print(f"    Message {message.message_id} reached EdgeNode {edge_id} ({range_type} range: {distance:.1f}m)")
            print(f"    Hop chain: {' -> '.join(message.propagation_path)} -> {edge_id}")
            print(f"    Total hops: {message.hop_count}")
            
            # Stop propagation when edge node is reached
            hop_count_stats[message.hop_count] += 1
            return True
    
    # Continue propagation to nearby vehicles (with enhanced range)
    vehicles_in_range = find_vehicles_in_range(current_position, exclude_vehicle=current_vehicle_id)
    
    print(f"    Scanning {len(vehicles_in_range)} vehicles within {V2V_COMMUNICATION_RANGE}m range")
    
    propagation_successful = False
    for vehicle_id, distance, vehicle_pos in vehicles_in_range:
        
        # Skip if this vehicle already received this message
        if vehicle_id in message.propagation_path:
            continue
        
        # Skip stopped vehicles (they can't effectively relay)
        if vehicle_id in stopped_vehicles:
            continue
        
        # Create a new message instance for this hop
        next_message = V2VMessage(
            message.message_id,
            vehicle_id,  # New source for next hop
            message.message_type,
            message.payload,
            message.origin_location
        )
        next_message.hop_count = message.hop_count + 1
        next_message.propagation_path = message.propagation_path.copy()
        next_message.increment_hop(vehicle_id)
        
        range_type = "Extended" if distance > 80 else "Standard"
        print(f"    Hop {next_message.hop_count}: {current_vehicle_id} -> {vehicle_id} ({range_type}: {distance:.1f}m)")
        
        # Store message in global history
        if next_message.message_id not in message_history:
            message_history[next_message.message_id] = []
        message_history[next_message.message_id].append(next_message)
        
        # Continue propagation from this vehicle
        success = propagate_v2v_message(next_message, vehicle_id, vehicle_pos)
        if success:
            propagation_successful = True
            break  # Stop after first successful path to edge node
    
    return propagation_successful

def broadcast_emergency_alert(source_vehicle_id, accident_location, collision_pair):
    """Broadcast emergency alert using multi-hop V2V communication with ENHANCED RANGE"""
    
    source_position = get_vehicle_position(source_vehicle_id)
    if not source_position:
        return False
    
    # Create emergency message
    message_id = f"EMERGENCY_{uuid.uuid4().hex[:8]}"
    accident_id = f"accident_{len(accident_locations)}"
    
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
    
    print(f"\nENHANCED Multi-Hop V2V Emergency Broadcast")
    print(f"Source: {source_vehicle_id} at ({source_position[0]:.1f}, {source_position[1]:.1f})")
    print(f"Enhanced V2V Range: {V2V_COMMUNICATION_RANGE}m (increased from 80m)")
    print(f"Message ID: {message_id}")
    print(f"Accident ID: {accident_id}")
    
    # Store message in history
    message_history[message_id] = [message]
    
    # Start propagation with enhanced range
    success = propagate_v2v_message(message, source_vehicle_id, source_position)
    
    if success:
        print(f"Emergency alert successfully reached edge infrastructure with enhanced V2V range!")
        
        # Trigger ambulance response for successful emergency messages
        ambulances_dispatched = 0
        for vehicle_id in traci.vehicle.getIDList():
            if "ambulance" in vehicle_id.lower() and vehicle_id not in ambulance_rerouted:
                if reroute_ambulance_to_accident(vehicle_id, accident_location):
                    ambulance_rerouted.add(vehicle_id)
                    ambulances_dispatched += 1
        
        print(f"Emergency response: {ambulances_dispatched} ambulances dispatched")
        return True
    else:
        print(f"Warning: Emergency alert could not reach edge infrastructure even with enhanced range")
        return False

def load_routes_and_vtypes():
    """Load available routes and vehicle types from vehicles.rou.xml"""
    try:
        rou_file = "vehicles.rou.xml"
        tree = ET.parse(rou_file)
        root = tree.getroot()
        
        routes = [r.attrib['id'] for r in root.findall("route") 
                 if not r.attrib['id'].startswith('routeAmbulance')]
        vtypes = {v.attrib['id']: v for v in root.findall("vType")}
        
        print(f"Loaded {len(routes)} regular routes and {len(vtypes)} vehicle types")
        return routes, vtypes
    except Exception as e:
        print(f"Error loading routes: {e}")
        return [], {}

def detect_collisions():
    """Detect vehicle collisions based on position proximity"""
    vehicles = traci.vehicle.getIDList()
    positions = {}
    
    # Get all vehicle positions
    for vid in vehicles:
        if vid not in stopped_vehicles:  # Only check moving vehicles
            try:
                positions[vid] = traci.vehicle.getPosition(vid)
            except:
                continue
    
    # Check for collisions
    new_collisions = []
    for vid1, pos1 in positions.items():
        for vid2, pos2 in positions.items():
            if vid1 >= vid2:
                continue
                
            distance = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
            pair = tuple(sorted([vid1, vid2]))
            
            if distance < COLLISION_DISTANCE and pair not in collision_pairs:
                # Stop both vehicles
                for v in pair:
                    try:
                        traci.vehicle.setSpeed(v, 0)
                        traci.vehicle.setColor(v, (255, 0, 0, 255))  # Red for accident
                        stopped_vehicles.add(v)
                    except:
                        pass
                
                collision_pairs.add(pair)
                new_collisions.append((pair, pos1))
                
                sim_time = traci.simulation.getTime()
                print(f"\nCOLLISION DETECTED: {vid1} and {vid2}")
                print(f"Location: ({pos1[0]:.1f}, {pos1[1]:.1f})")
                print(f"Time: {sim_time:.1f}s")
    
    return new_collisions

def create_artificial_collision():
    """Create artificial collision for testing enhanced V2V communication"""
    vehicles = [v for v in traci.vehicle.getIDList() if v not in stopped_vehicles and "ambulance" not in v.lower()]
    
    if len(vehicles) >= 2:
        # Select two vehicles for artificial collision
        collision_vehicles = random.sample(vehicles, 2)
        
        # Get position of first vehicle for collision location
        collision_location = get_vehicle_position(collision_vehicles[0])
        
        if collision_location:
            # Stop both vehicles and mark as collided
            for v in collision_vehicles:
                try:
                    traci.vehicle.setSpeed(v, 0)
                    traci.vehicle.setColor(v, (255, 0, 0, 255))  # Red for accident
                    stopped_vehicles.add(v)
                except:
                    pass
            
            collision_pair = tuple(sorted(collision_vehicles))
            collision_pairs.add(collision_pair)
            
            print(f"\nARTIFICIAL COLLISION CREATED for Enhanced V2V Testing")
            print(f"Vehicles: {collision_vehicles[0]} and {collision_vehicles[1]}")
            print(f"Location: ({collision_location[0]:.1f}, {collision_location[1]:.1f})")
            
            return [(collision_pair, collision_location)]
    
    return []

def find_closest_edge_to_position(x, y):
    """Find the edge closest to given coordinates"""
    try:
        all_edges = traci.edge.getIDList()
        min_distance = float('inf')
        closest_edge = None
        
        for edge_id in all_edges:
            if edge_id.startswith(':'):  # Skip internal edges
                continue
                
            try:
                edge_shape = traci.edge.getShape(edge_id)
                for edge_point in edge_shape:
                    edge_x, edge_y = edge_point
                    distance = math.hypot(x - edge_x, y - edge_y)
                    if distance < min_distance:
                        min_distance = distance
                        closest_edge = edge_id
            except:
                continue
        
        return closest_edge, min_distance
    except Exception as e:
        print(f"Error finding closest edge: {e}")
        return None, float('inf')

def reroute_ambulance_to_accident(ambulance_id, accident_location):
    """Enhanced ambulance rerouting with multiple strategies"""
    try:
        if ambulance_id not in traci.vehicle.getIDList():
            return False
        
        ambulance_pos = traci.vehicle.getPosition(ambulance_id)
        accident_x, accident_y = accident_location
        
        print(f"DISPATCHING AMBULANCE: {ambulance_id}")
        print(f"From: ({ambulance_pos[0]:.1f}, {ambulance_pos[1]:.1f})")
        print(f"To: ({accident_x:.1f}, {accident_y:.1f})")
        
        success = False
        
        # Strategy 1: Emergency rerouting
        try:
            traci.vehicle.rerouteTraveltime(ambulance_id)
            print(f"Emergency reroute activated for {ambulance_id}")
            success = True
        except:
            pass
        
        # Strategy 2: Change target edge
        target_edge, _ = find_closest_edge_to_position(accident_x, accident_y)
        if target_edge:
            try:
                traci.vehicle.changeTarget(ambulance_id, target_edge)
                print(f"Target changed to: {target_edge}")
                success = True
            except Exception as e:
                print(f"Target change failed: {e}")
        
        # Always activate emergency mode
        try:
            # Increase speed and priority
            current_speed = traci.vehicle.getMaxSpeed(ambulance_id)
            new_speed = min(current_speed * EMERGENCY_SPEED_BOOST, 25.0)
            traci.vehicle.setMaxSpeed(ambulance_id, new_speed)
            
            # Set emergency appearance
            traci.vehicle.setColor(ambulance_id, (255, 255, 0, 255))  # Yellow emergency
            traci.vehicle.setLength(ambulance_id, 8.0)
            traci.vehicle.setWidth(ambulance_id, 2.5)
            
            # Reduce following behavior for emergency passage
            traci.vehicle.setTau(ambulance_id, 0.5)
            traci.vehicle.setMinGap(ambulance_id, 1.0)
            traci.vehicle.setSpeedMode(ambulance_id, 0)  # Ignore speed limits temporarily
            
            print(f"Emergency mode: Speed={new_speed:.1f}m/s, Priority=HIGH")
            
        except Exception as e:
            print(f"Emergency mode warning: {e}")
        
        return success
        
    except Exception as e:
        print(f"Ambulance rerouting error: {e}")
        return False

def monitor_ambulance_response():
    """Monitor ambulance response to accidents"""
    for ambulance_id in [v for v in traci.vehicle.getIDList() if "ambulance" in v.lower()]:
        if ambulance_id not in ambulance_rerouted:
            continue
            
        try:
            ambulance_pos = traci.vehicle.getPosition(ambulance_id)
            ambulance_speed = traci.vehicle.getSpeed(ambulance_id)
            
            # Check proximity to any accident
            for accident_id, accident_pos in accident_locations.items():
                distance = math.hypot(ambulance_pos[0] - accident_pos[0], 
                                    ambulance_pos[1] - accident_pos[1])
                
                if distance < AMBULANCE_RESPONSE_DISTANCE:
                    print(f"AMBULANCE RESPONSE: {ambulance_id} reached accident scene")
                    print(f"Distance to accident: {distance:.1f}m")
                    print(f"Response time: {traci.simulation.getTime():.1f}s")
                    
                    # Slow down ambulance at scene
                    traci.vehicle.setSpeed(ambulance_id, 2.0)
                    traci.vehicle.setColor(ambulance_id, (0, 255, 0, 255))  # Green for arrived
                    
                    # Remove from active response list
                    if ambulance_id in ambulance_rerouted:
                        ambulance_rerouted.remove(ambulance_id)
                        
        except Exception as e:
            continue

def spawn_initial_vehicles():
    """Spawn initial vehicles in the simulation"""
    routes, vtypes = load_routes_and_vtypes()
    
    if not routes or not vtypes:
        print("Warning: No routes or vehicle types loaded")
        return
    
    # Spawn regular vehicles
    num_vehicles = 8
    regular_vtypes = [v for v in vtypes.keys() if v != "ambulance"]
    
    for i in range(num_vehicles):
        vid = f"vehicle_{i}"
        route = random.choice(routes)
        vtype = random.choice(regular_vtypes)
        
        try:
            traci.vehicle.add(vid, routeID=route, typeID=vtype)
            
            # Set random speed within limits
            max_speed = float(vtypes[vtype].attrib.get("maxSpeed", 13.89))
            speed = random.uniform(max_speed * 0.5, max_speed * 0.9)
            traci.vehicle.setSpeed(vid, speed)
            
            print(f"Spawned {vid} on {route} as {vtype} (speed: {speed:.1f}m/s)")
            
        except Exception as e:
            print(f"Failed to spawn {vid}: {e}")

def print_communication_statistics():
    """Print detailed V2V communication statistics"""
    print("\n" + "=" * 60)
    print("ENHANCED V2V COMMUNICATION STATISTICS")
    print("=" * 60)
    
    print(f"Enhanced V2V Communication Range: {V2V_COMMUNICATION_RANGE}m (increased from 80m)")
    print(f"Total unique messages: {len(message_history)}")
    print(f"Total message hops: {sum(hop_count_stats.values())}")
    
    print("\nHop count distribution:")
    for hops in sorted(hop_count_stats.keys()):
        print(f"  {hops} hops: {hop_count_stats[hops]} messages")
    
    print("\nEdge node statistics:")
    for edge_id, edge_info in edge_nodes.items():
        print(f"  {edge_id}:")
        print(f"    Messages received: {edge_info['total_messages_received']}")
        print(f"    Unique accidents: {len(edge_info['unique_accidents_reported'])}")
        print(f"    Position: {edge_info['position']}")
        print(f"    Communication range: {V2V_COMMUNICATION_RANGE}m")

def run_simulation():
    """Main simulation with ENHANCED multi-hop V2V and edge nodes"""
    try:
        # Start SUMO
        sumo_binary = sumolib.checkBinary("sumo-gui")
        traci.start([
            sumo_binary, "-c", "simulation.sumocfg",
            "--collision.action", "none",
            "--collision.check-junctions", "true",
            "--step-length", "0.1",
            "--start"
        ])
        
        print("ENHANCED Multi-Hop V2V Emergency Response Simulation Started")
        print("=" * 70)
        print(f"ENHANCED V2V Communication Range: {V2V_COMMUNICATION_RANGE}m (increased from 80m)")
        print(f"Maximum Hops: {MAX_HOP_COUNT}")
        print(f"Range Improvement: +{V2V_COMMUNICATION_RANGE - 80.0:.0f}m (+{((V2V_COMMUNICATION_RANGE/80.0 - 1) * 100):.0f}%)")
        print("=" * 70)
        
        # Initialize edge nodes
        initialize_edge_nodes()
        
        # Initialize mock network components
        mobility_helper = mock_ns3.MobilityHelper()
        wifi_helper = mock_ns3.WifiHelper()
        stack_helper = mock_ns3.InternetStackHelper()
        address_helper = mock_ns3.Ipv4AddressHelper()
        
        # Spawn initial vehicles
        spawn_initial_vehicles()
        
        step = 0
        accidents_handled = 0
        
        # Main simulation loop
        while step < MAX_STEPS:
            traci.simulationStep()
            step += 1
            
            # Handle new vehicle departures
            departed_ids = traci.simulation.getDepartedIDList()
            for vid in departed_ids:
                if vid not in vehicle_nodes:
                    # Create mock network node
                    vehicle_nodes[vid] = {
                        'node': f"node_{vid}",
                        'ip': f"10.1.1.{len(vehicle_nodes) + 1}",
                        'created_at': step,
                        'v2v_range': V2V_COMMUNICATION_RANGE  # Store enhanced range
                    }
                    
                    vehicle_type = "ambulance" if "ambulance" in vid.lower() else "vehicle"
                    print(f"Enhanced V2V node created for {vehicle_type} {vid} (range: {V2V_COMMUNICATION_RANGE}m)")
            
            # Detect collisions and trigger enhanced multi-hop communication
            new_collisions = detect_collisions()
            
            # Add artificial collision after some time for testing enhanced V2V
            if step == 200 and len(collision_pairs) == 0:
                artificial_collisions = create_artificial_collision()
                for collision_pair, location in artificial_collisions:
                    accident_id = f"enhanced_test_accident_{accidents_handled}"
                    accident_locations[accident_id] = location
                    accidents_handled += 1
                    
                    # Broadcast emergency alert using enhanced multi-hop V2V
                    source_vehicle = collision_pair[0]
                    success = broadcast_emergency_alert(source_vehicle, location, collision_pair)
                    
                    print(f"\nENHANCED TEST EMERGENCY RESPONSE INITIATED")
                    print(f"Accident ID: {accident_id}")
                    print(f"Vehicles involved: {', '.join(collision_pair)}")
                    print(f"Enhanced multi-hop broadcast: {'SUCCESS' if success else 'FAILED'}")
            
            # Handle new accidents with enhanced multi-hop V2V
            for collision_pair, location in new_collisions:
                accident_id = f"enhanced_accident_{accidents_handled}"
                accident_locations[accident_id] = location
                accidents_handled += 1
                
                # Broadcast emergency alert using enhanced multi-hop V2V
                source_vehicle = collision_pair[0]  # Use first vehicle as source
                success = broadcast_emergency_alert(source_vehicle, location, collision_pair)
                
                print(f"\nENHANCED EMERGENCY RESPONSE INITIATED")
                print(f"Accident ID: {accident_id}")
                print(f"Vehicles involved: {', '.join(collision_pair)}")
                print(f"Enhanced multi-hop broadcast: {'SUCCESS' if success else 'FAILED'}")
            
            # Monitor ambulance responses
            monitor_ambulance_response()
            
            # Cleanup departed vehicles
            departed = traci.simulation.getArrivedIDList()
            for vid in departed:
                if vid in vehicle_nodes:
                    del vehicle_nodes[vid]
            
            # Progress indicator with enhanced range info
            if step % 100 == 0:
                current_vehicles = len(traci.vehicle.getIDList())
                total_messages = sum(len(msgs) for msgs in message_history.values())
                edge_messages = sum(info['total_messages_received'] for info in edge_nodes.values())
                print(f"Step {step}: {current_vehicles} vehicles, {len(collision_pairs)} accidents, {total_messages} V2V hops, {edge_messages} edge msgs (Enhanced {V2V_COMMUNICATION_RANGE}m range)")
            
            time.sleep(0.05)  # Slow down for visualization
        
        # Final statistics
        print("\n" + "=" * 70)
        print("ENHANCED V2V SIMULATION COMPLETE")
        print("=" * 70)
        print(f"Enhanced V2V Range: {V2V_COMMUNICATION_RANGE}m (vs original 80m)")
        print(f"Range improvement: +{((V2V_COMMUNICATION_RANGE/80.0 - 1) * 100):.0f}%")
        print(f"Total simulation steps: {step}")
        print(f"Accidents detected: {len(collision_pairs)}")
        print(f"Total V2V message hops: {sum(len(msgs) for msgs in message_history.values())}")
        print(f"Ambulances dispatched: {len(ambulance_rerouted)}")
        print(f"Emergency responses: {accidents_handled}")
        
        # Print detailed enhanced communication statistics
        print_communication_statistics()
        
    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        try:
            traci.close()
        except:
            pass
        print("Enhanced multi-hop V2V emergency response simulation ended")

if __name__ == "__main__":
    run_simulation()
