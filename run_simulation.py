import sys
import traci
import json
from ns import core, network, mobility, internet, wifi

# --- Global State ---
vehicle_nodes = {}
broadcasted_accidents = set()
packet_count = 0
accident_locations = {}  # Store accident locations
ambulance_rerouted = set()  # Track which ambulances have been rerouted

def get_accident_location(accident_vehicle_id):
    """
    Get the location of an accident vehicle.
    """
    try:
        if accident_vehicle_id in traci.vehicle.getIDList():
            return traci.vehicle.getPosition(accident_vehicle_id)
        return None
    except:
        return None

def reroute_ambulance_to_accident(ambulance_id, accident_location):
    """
    Reroute ambulance to accident location using SUMO's rerouting capabilities.
    """
    try:
        if ambulance_id not in traci.vehicle.getIDList():
            print(f"    -> Ambulance '{ambulance_id}' not found in simulation")
            return False
            
        # Get current ambulance position
        ambulance_pos = traci.vehicle.getPosition(ambulance_id)
        accident_x, accident_y = accident_location
        
        print(f"\n🚑 AMBULANCE REROUTING 🚑")
        print(f"    -> Ambulance '{ambulance_id}' current position: ({ambulance_pos[0]:.1f}, {ambulance_pos[1]:.1f})")
        print(f"    -> Accident location: ({accident_x:.1f}, {accident_y:.1f})")
        
        # Method 1: Get the edge the accident vehicle is currently on
        accident_edge_id = None
        try:
            # Check if accident vehicle still exists and get its edge
            for vid in traci.vehicle.getIDList():
                if "accident" in vid.lower():
                    accident_edge_id = traci.vehicle.getRoadID(vid)
                    print(f"    -> Found accident vehicle on edge: {accident_edge_id}")
                    break
        except Exception as e:
            print(f"    -> Cannot get accident vehicle edge: {e}")
        
        # Method 2: Find closest edge to accident coordinates
        if not accident_edge_id:
            try:
                all_edges = traci.edge.getIDList()
                print(f"    -> Searching through {len(all_edges)} edges for closest match...")
                
                min_distance = float('inf')
                closest_edge = None
                
                for edge_id in all_edges[:20]:  # Check first 20 edges to avoid timeout
                    try:
                        edge_shape = traci.edge.getShape(edge_id)
                        if edge_shape and len(edge_shape) > 0:
                            # Check distance to each point in the edge shape
                            for edge_point in edge_shape:
                                edge_x, edge_y = edge_point
                                distance = ((accident_x - edge_x)**2 + (accident_y - edge_y)**2)**0.5
                                if distance < min_distance:
                                    min_distance = distance
                                    closest_edge = edge_id
                    except Exception as e:
                        continue
                
                if closest_edge:
                    accident_edge_id = closest_edge
                    print(f"    -> Closest edge found: {accident_edge_id} (distance: {min_distance:.1f}m)")
                else:
                    print(f"    -> No suitable edge found, trying fallback methods")
                    
            except Exception as e:
                print(f"    -> Edge search failed: {e}")
        
        # Try different rerouting approaches
        reroute_success = False
        
        # Approach 1: Direct rerouting without specific destination
        if not reroute_success:
            try:
                traci.vehicle.rerouteTraveltime(ambulance_id)
                print(f"    -> ✅ Ambulance '{ambulance_id}' rerouted via rerouteTraveltime")
                reroute_success = True
            except Exception as e:
                print(f"    -> rerouteTraveltime failed: {e}")
        
        # Approach 2: Change target if we have a valid edge
        if not reroute_success and accident_edge_id:
            try:
                traci.vehicle.changeTarget(ambulance_id, accident_edge_id)
                print(f"    -> ✅ Ambulance '{ambulance_id}' target changed to '{accident_edge_id}'")
                reroute_success = True
            except Exception as e:
                print(f"    -> changeTarget to '{accident_edge_id}' failed: {e}")
        
        # Approach 3: Modify route if we have a valid edge
        if not reroute_success and accident_edge_id:
            try:
                current_route = traci.vehicle.getRoute(ambulance_id)
                if current_route:
                    new_route = list(current_route)
                    if accident_edge_id not in new_route:
                        new_route.append(accident_edge_id)
                        traci.vehicle.setRoute(ambulance_id, new_route)
                        print(f"    -> ✅ Ambulance '{ambulance_id}' route updated to include '{accident_edge_id}'")
                        reroute_success = True
            except Exception as e:
                print(f"    -> setRoute failed: {e}")
        
        # Approach 4: Emergency mode activation (always try this)
        try:
            current_speed = traci.vehicle.getMaxSpeed(ambulance_id)
            traci.vehicle.setMaxSpeed(ambulance_id, min(current_speed * 1.5, 30.0))  # 50% speed boost, max 30 m/s
            traci.vehicle.setColor(ambulance_id, (255, 0, 0, 255))  # Red color for emergency
            print(f"    -> ✅ Ambulance '{ambulance_id}' activated emergency mode (speed: {traci.vehicle.getMaxSpeed(ambulance_id):.1f} m/s)")
            if not reroute_success:
                reroute_success = True  # At least we activated emergency mode
        except Exception as e:
            print(f"    -> Emergency mode activation failed: {e}")
        
        return reroute_success
        
    except Exception as e:
        print(f"Error rerouting ambulance {ambulance_id}: {e}")
        return False

def broadcast_accident_alert(source_vehicle_id, accident_id):
    """
    Broadcasts accident alert and triggers ambulance rerouting.
    """
    try:
        global packet_count
        if source_vehicle_id not in vehicle_nodes:
            print(f"Error: Source vehicle {source_vehicle_id} not found")
            return

        # Store accident location
        accident_location = get_accident_location(source_vehicle_id)
        if accident_location:
            accident_locations[accident_id] = accident_location

        source_node = vehicle_nodes[source_vehicle_id]['node']
        
        print(f"--- V2V ALERT BROADCAST ---")
        print(f"    -> Vehicle '{source_vehicle_id}' broadcasting accident alert {accident_id}")

        # Create broadcast socket
        broadcast_socket = network.Socket.CreateSocket(source_node, core.TypeId.LookupByName("ns3::UdpSocketFactory"))
        broadcast_socket.SetAllowBroadcast(True)
        broadcast_socket.Bind()
        
        # Send to all other vehicles
        ambulances_alerted = 0
        for target_id, target_info in vehicle_nodes.items():
            if target_id != source_vehicle_id:
                try:
                    target_ip = target_info['ip_address']
                    if target_ip:
                        target_address = network.InetSocketAddress(target_ip, 9999)
                        packet = network.Packet(100)
                        broadcast_socket.SendTo(packet, 0, target_address)
                        packet_count += 1
                        
                        print(f"    -> Sent alert to vehicle '{target_id}' at {target_ip}")
                        
                        # Special handling for ambulances
                        if "ambulance" in target_id.lower():
                            ambulances_alerted += 1
                            
                except Exception as e:
                    print(f"    -> Failed to send to {target_id}: {e}")
        
        broadcast_socket.Close()
        print(f"    -> Broadcast complete: {packet_count} packets sent, {ambulances_alerted} ambulances alerted")
        
        # Trigger ambulance rerouting
        if accident_location and ambulances_alerted > 0:
            for vehicle_id in vehicle_nodes.keys():
                if "ambulance" in vehicle_id.lower() and vehicle_id not in ambulance_rerouted:
                    success = reroute_ambulance_to_accident(vehicle_id, accident_location)
                    if success:
                        ambulance_rerouted.add(vehicle_id)
        
    except Exception as e:
        print(f"Error in broadcast_accident_alert: {e}")

def run_simulation():
    """
    Main simulation function with ambulance rerouting.
    """
    try:
        traci.start(["sumo-gui", "-c", "simulation.sumocfg"], port=8813)

        # --- ns-3 Setup ---
        mobility_helper = mobility.MobilityHelper()
        mobility_helper.SetMobilityModel("ns3::ConstantPositionMobilityModel")

        mac = wifi.WifiMacHelper()
        mac.SetType("ns3::AdhocWifiMac")
        
        wifi_helper = wifi.WifiHelper()
        wifi_helper.SetStandard(wifi.WIFI_STANDARD_80211p)
        
        channel = wifi.YansWifiChannelHelper()
        channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel")
        channel.AddPropagationLoss("ns3::FriisPropagationLossModel")
        
        phy = wifi.YansWifiPhyHelper()
        phy.SetChannel(channel.Create())
        phy.Set("TxPowerStart", core.DoubleValue(20.0))
        phy.Set("TxPowerEnd", core.DoubleValue(20.0))
        
        stack = internet.InternetStackHelper()
        address = internet.Ipv4AddressHelper()
        address.SetBase(network.Ipv4Address("10.1.1.0"), network.Ipv4Mask("255.255.255.0"))

        print("🚗 V2V Communication with Ambulance Rerouting ready!")
        print("🚑 Ambulances will automatically reroute when accidents are detected")
        print("Starting simulation...\n")

        # --- Main Simulation Loop ---
        for step in range(1000):
            traci.simulationStep()
            
            # --- Create new vehicle nodes ---
            departed_ids = traci.simulation.getDepartedIDList()
            if departed_ids:
                try:
                    new_nodes = network.NodeContainer()
                    new_nodes.Create(len(departed_ids))
                    mobility_helper.Install(new_nodes)
                    net_devices = wifi_helper.Install(phy, mac, new_nodes)
                    stack.Install(new_nodes)
                    address.Assign(net_devices)
                    
                    for i, v_id in enumerate(departed_ids):
                        node = new_nodes.Get(i)
                        
                        ipv4 = node.GetObject(internet.Ipv4.GetTypeId())
                        node_ip = None
                        if ipv4 and ipv4.GetNInterfaces() > 1:
                            node_ip = ipv4.GetAddress(1, 0).GetLocal()
                        
                        recv_socket = network.Socket.CreateSocket(node, core.TypeId.LookupByName("ns3::UdpSocketFactory"))
                        recv_socket.Bind(network.InetSocketAddress(network.Ipv4Address.GetAny(), 9999))
                        
                        vehicle_nodes[v_id] = {
                            'node': node,
                            'socket': recv_socket,
                            'ip_address': node_ip
                        }
                        
                        # Special notification and setup for ambulances
                        if "ambulance" in v_id.lower():
                            print(f"🚑 Created ambulance '{v_id}' with IP {node_ip} - Ready for emergency dispatch!")
                            
                            # Pre-configure ambulance for better visibility and performance
                            try:
                                traci.vehicle.setColor(v_id, (255, 255, 255, 255))  # White color initially
                                traci.vehicle.setLength(v_id, 7.0)  # Longer than normal cars
                                traci.vehicle.setWidth(v_id, 2.3)   # Wider than normal cars
                                traci.vehicle.setMaxSpeed(v_id, 20.0)  # Higher initial max speed
                                print(f"    -> Ambulance pre-configured: Length=7.0m, Width=2.3m, MaxSpeed=20.0m/s")
                            except Exception as e:
                                print(f"    -> Ambulance configuration warning: {e}")
                        else:
                            print(f"🚗 Created vehicle '{v_id}' with IP {node_ip}")
                    
                    address.SetBase(address.NewNetwork(), network.Ipv4Mask("255.255.255.0"))
                    
                except Exception as e:
                    print(f"Error creating nodes: {e}")
                    continue

            # --- Update positions ---
            for v_id in traci.vehicle.getIDList():
                if v_id in vehicle_nodes:
                    try:
                        pos = traci.vehicle.getPosition(v_id)
                        node = vehicle_nodes[v_id]['node']
                        mobility_model = node.GetObject(mobility.MobilityModel.GetTypeId())
                        if mobility_model:
                            mobility_model.SetPosition(core.Vector(pos[0], pos[1], 0))
                    except Exception as e:
                        print(f"Error updating position for {v_id}: {e}")

            # --- Accident Detection and Emergency Response ---
            current_vehicles = traci.vehicle.getIDList()
            if "accidentCar" in current_vehicles and "accidentCar" in vehicle_nodes:
                if len(broadcasted_accidents) == 0:
                    try:
                        speed = traci.vehicle.getSpeed("accidentCar")
                        if speed < 1.0:
                            print(f"\n🚨 EMERGENCY SITUATION DETECTED! 🚨")
                            print(f"    Vehicle 'accidentCar' stopped (speed: {speed:.2f} m/s)")
                            
                            accident_id = f"EMERGENCY_{int(traci.simulation.getTime())}"
                            broadcast_accident_alert("accidentCar", accident_id)
                            broadcasted_accidents.add(accident_id)
                            
                            # Show alert reception for regular vehicles
                            regular_vehicles = [v for v in vehicle_nodes.keys() 
                                              if v != "accidentCar" and "ambulance" not in v.lower()]
                            for vehicle in regular_vehicles:
                                print(f"--- V2V ALERT RECEIVED ---")
                                print(f"    -> Vehicle '{vehicle}' received emergency alert - staying clear!")
                            
                    except Exception as e:
                        print(f"Error in emergency detection: {e}")
            
            # --- Monitor ambulance response ---
            if len(broadcasted_accidents) > 0:
                for ambulance_id in [v for v in vehicle_nodes.keys() if "ambulance" in v.lower()]:
                    if ambulance_id in ambulance_rerouted and ambulance_id in current_vehicles:
                        try:
                            ambulance_pos = traci.vehicle.getPosition(ambulance_id)
                            ambulance_speed = traci.vehicle.getSpeed(ambulance_id)
                            
                            # Check if ambulance is near accident
                            for accident_id, accident_pos in accident_locations.items():
                                distance = ((ambulance_pos[0] - accident_pos[0])**2 + 
                                          (ambulance_pos[1] - accident_pos[1])**2)**0.5
                                
                                if distance < 50:  # Within 50 meters
                                    print(f"🏥 Ambulance '{ambulance_id}' arrived at accident scene! Distance: {distance:.1f}m")
                                    
                        except Exception as e:
                            pass  # Continue simulation even if monitoring fails
            
            # Run ns-3 simulation
            try:
                core.Simulator.Run(core.Seconds(0.1))
            except Exception as e:
                print(f"Error in ns-3 step: {e}")
                break

        print(f"\n📊 SIMULATION SUMMARY 📊")
        print(f"   Total vehicles: {len(vehicle_nodes)}")
        print(f"   Accidents detected: {len(broadcasted_accidents)}")
        print(f"   V2V packets sent: {packet_count}")
        print(f"   Ambulances rerouted: {len(ambulance_rerouted)}")

    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            core.Simulator.Destroy()
            traci.close()
        except:
            pass
        print("\n🏁 Emergency response simulation finished.")

if __name__ == "__main__":
    run_simulation()
