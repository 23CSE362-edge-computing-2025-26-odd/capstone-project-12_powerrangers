import sys
import os
import time
import random
import xml.etree.ElementTree as ET
import math

# -------------------- SUMO PATH SETUP --------------------
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)

import traci
import sumolib

# ----------------------------
# SUMO setup
sumoBinary = sumolib.checkBinary("sumo-gui")
sumoConfig = "simulation.sumocfg"

# ----------------------------
# Parse routes and vehicle types from vehicles.rou.xml
rou_file = "vehicles.rou.xml"
tree = ET.parse(rou_file)
root = tree.getroot()

routes = [r.attrib['id'] for r in root.findall("route")]
vtypes = {v.attrib['id']: v for v in root.findall("vType")}

print("Loaded routes:", routes)
print("Loaded vehicle types:", list(vtypes.keys()))

# ----------------------------
NUM_VEHICLES = 8
NUM_AMBULANCES = 3
step = 0
MAX_STEPS = 500
COLLISION_DISTANCE = 5.0  # meters threshold

# Track permanently stopped vehicles and reported collisions
stopped_vehicles = set()
reported_collisions = set()

traci.start([
    sumoBinary, "-c", sumoConfig,
    "--collision.action", "none",
    "--collision.check-junctions", "true",
    "--ignore-route-errors", "true"
], port=8813)   # 👈 use port param here


# ----------------------------
# Spawn ambulances at fixed parking edges using dedicated routes
ambulance_parking_routes = {
    "ambulance0": "routeAmbulance0",
    "ambulance1": "routeAmbulance1",
    "ambulance2": "routeAmbulance2"
}

for vid, route in ambulance_parking_routes.items():
    traci.vehicle.add(vid, routeID=route, typeID="ambulance", depart=0)
    traci.vehicle.setSpeed(vid, 0)
    traci.vehicle.setEmergencyDecel(vid, 1000)
    traci.vehicle.setTau(vid, 0)
    traci.vehicle.setMinGap(vid, 0)

    # Use the edge from the single-edge route
    edgeID = traci.route.getEdges(route)[0]
    traci.vehicle.setStop(vid, edgeID=edgeID, pos=0, duration=1e6)

    stopped_vehicles.add(vid)
    print(f"Ambulance {vid} spawned on {route} ({edgeID}) in parking mode")

# ----------------------------
# Spawn regular vehicles at start
for i in range(NUM_VEHICLES):
    vid = f"veh{i}"
    r = random.choice(routes)
    t = random.choice([v for v in vtypes.keys() if v != "ambulance"])
    traci.vehicle.add(vid, routeID=r, typeID=t, depart=0)
    traci.vehicle.setEmergencyDecel(vid, 1000)
    traci.vehicle.setTau(vid, 0)
    traci.vehicle.setMinGap(vid, 0)

    # Random speed
    max_speed = float(vtypes[t].attrib.get("maxSpeed", 13.9))
    speed = random.uniform(max_speed * 0.3, max_speed)
    traci.vehicle.setSpeed(vid, speed)
    print(f"Added {vid} on route {r} as {t} at speed {speed:.1f} m/s")

# ----------------------------
while step < MAX_STEPS:
    traci.simulationStep()
    step += 1
    time.sleep(0.5)

    # Get positions of all vehicles
    vehicles = traci.vehicle.getIDList()
    positions = {vid: traci.vehicle.getPosition(vid) for vid in vehicles}

    # Check for collisions by threshold and report each unique pair only once
    for vid1, pos1 in positions.items():
        for vid2, pos2 in positions.items():
            if vid1 >= vid2:
                continue
            distance = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
            pair = tuple(sorted([vid1, vid2]))
            if distance < COLLISION_DISTANCE and pair not in reported_collisions:
                # Stop involved vehicles permanently
                for v in pair:
                    traci.vehicle.setSpeed(v, 0)
                    stopped_vehicles.add(v)

                # Log full accident info (absolute X, Y coordinates)
                x, y = pos1  # pos1 is already absolute coords from traci.vehicle.getPosition
                sim_time = traci.simulation.getTime()
                print(f"⚠️ Accident detected between {vid1} and {vid2} "
                      f"at location=({x:.2f}, {y:.2f}), time={sim_time:.1f}s")

                # Save for OMNeT++ integration (DENM message input)
                reported_collisions.add(pair)

# ----------------------------
traci.close()
