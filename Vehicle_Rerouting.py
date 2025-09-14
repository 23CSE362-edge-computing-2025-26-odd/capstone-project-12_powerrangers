# Vehicle_Rerouting.py
import math
import random
import traci

# ---------------------------- Parameters ----------------------------
NUM_ANTS = 12
MAX_ANT_HOPS = 30
alpha = 1.0
beta = 2.0
evaporation_rate = 0.1

# Shared state
stopped_vehicles = set()
blocked_edges = set()
pheromone = {}

# -------------------- DEFAULT HELPER FUNCTIONS --------------------
def default_get_successor_edges(edge):
    """Return outgoing edges from this edge (SUMO traci wrapper)."""
    try:
        return list(traci.edge.getOutgoing(edge))
    except Exception:
        return []

def default_get_fuzzy_congestion(edge):
    """Return occupancy [0,1] for this edge."""
    try:
        occ = traci.edge.getLastStepOccupancy(edge)
        return max(0.0, min(1.0, float(occ)))
    except Exception:
        return 1.0

def update_pheromone(edge, delta):
    pheromone[edge] = pheromone.get(edge, 1.0) * (1 - evaporation_rate) + delta

def select_next_edge(current, candidate_edges, get_fuzzy_congestion):
    available = [e for e in candidate_edges if e not in blocked_edges]
    if not available:
        return None
    scores = []
    for e in available:
        tau = pheromone.get(e, 1.0) ** alpha
        eta = (1.0 / (get_fuzzy_congestion(e) + 1e-6)) ** beta
        scores.append(tau * eta)
    total = sum(scores)
    if total <= 0:
        return random.choice(available)
    probs = [s / total for s in scores]
    return random.choices(available, weights=probs, k=1)[0]

def get_path_from_ants(start, end, get_successor_edges, get_fuzzy_congestion):
    """Return best path (list of edge ids) from start to end using ANT sampling."""
    if start == end:
        return [start]
    best_path = None
    best_score = float('inf')
    for _ in range(NUM_ANTS):
        path = [start]
        current = start
        visited = set([start])
        hops = 0
        while current != end and hops < MAX_ANT_HOPS:
            successor_edges = get_successor_edges(current)
            if not successor_edges:
                break
            next_edge = select_next_edge(current, successor_edges, get_fuzzy_congestion)
            if next_edge is None or next_edge in visited or next_edge in blocked_edges:
                break
            path.append(next_edge)
            visited.add(next_edge)
            current = next_edge
            hops += 1
        if current == end:
            score = sum(get_fuzzy_congestion(e) for e in path)
            if score < best_score:
                best_score = score
                best_path = path
    if best_path:
        delta = 1.0 / (1.0 + best_score)
        for e in best_path:
            update_pheromone(e, delta)
    return best_path

# -------------------- VEHICLE AFFECTED CHECK --------------------
def is_vehicle_affected(traci_instance, vid, lookahead=3):
    """Return True if vehicle is on or will soon be on a blocked edge."""
    try:
        cur_edge = traci_instance.vehicle.getRoadID(vid)
        if not cur_edge or cur_edge.startswith(':'):
            return False
        if cur_edge in blocked_edges:
            return True
        route = traci_instance.vehicle.getRoute(vid)
        if not route:
            return False
        try:
            idx = route.index(cur_edge)
        except ValueError:
            idx = 0
        for i in range(1, min(lookahead + 1, len(route) - idx)):
            if route[idx + i] in blocked_edges:
                return True
    except Exception:
        return False
    return False

# -------------------- VEHICLE REROUTING --------------------
def reroute_vehicles(traci_instance, get_successor_edges=None, get_fuzzy_congestion=None):
    """Reroute all vehicles affected by blocked edges"""
    if get_successor_edges is None:
        get_successor_edges = default_get_successor_edges
    if get_fuzzy_congestion is None:
        get_fuzzy_congestion = default_get_fuzzy_congestion

    reroute_count = 0
    for vid in traci_instance.vehicle.getIDList():
        if vid in stopped_vehicles or "ambulance" in vid:
            continue
        if is_vehicle_affected(traci_instance, vid):
            try:
                start_edge = traci_instance.vehicle.getRoadID(vid)
                if not start_edge or start_edge.startswith(':'):
                    continue
                current_route = traci_instance.vehicle.getRoute(vid)
                if not current_route:
                    continue
                end_edge = current_route[-1]
                new_route = get_path_from_ants(start_edge, end_edge, get_successor_edges, get_fuzzy_congestion)
                if new_route:
                    try:
                        traci_instance.vehicle.setRoute(vid, new_route)
                        print(f"Vehicle {vid} rerouted -> {' -> '.join(new_route)}")
                        reroute_count += 1
                    except Exception as e:
                        print(f"Failed to set route for {vid}: {e}")
                else:
                    traci_instance.vehicle.setSpeed(vid, 0)
                    stopped_vehicles.add(vid)
                    print(f"⚠ No alternate path for {vid}; stopped.")
            except Exception as e:
                print(f"Error processing vehicle {vid}: {e}")
    return reroute_count

# -------------------- ACCIDENT HANDLING --------------------
def handle_accident(traci_instance, vid1, vid2, get_successor_edges=None, get_fuzzy_congestion=None):
    """Stop vehicles, block edges, and reroute affected vehicles"""
    if get_successor_edges is None:
        get_successor_edges = default_get_successor_edges
    if get_fuzzy_congestion is None:
        get_fuzzy_congestion = default_get_fuzzy_congestion

    try:
        e1 = traci_instance.vehicle.getRoadID(vid1)
        e2 = traci_instance.vehicle.getRoadID(vid2)
        newly_blocked = []
        for e in [e1, e2]:
            if e and not e.startswith(':') and e not in blocked_edges:
                blocked_edges.add(e)
                newly_blocked.append(e)

        if newly_blocked:
            print(f"Blocking edges due to accident: {newly_blocked}")
        print(f"Accident between {vid1} and {vid2}")

        # Stop accident vehicles
        for v in (vid1, vid2):
            try:
                traci_instance.vehicle.setSpeed(v, 0)
                traci_instance.vehicle.setColor(v, (255, 0, 0, 255))
                traci_instance.vehicle.setEmergencyDecel(v, 0.0)
            except Exception as e:
                print(f"Error modifying vehicle {v}: {e}")
        stopped_vehicles.update([vid1, vid2])

        # Stop vehicles already on blocked edges
        for vid in traci_instance.vehicle.getIDList():
            try:
                if traci_instance.vehicle.getRoadID(vid) in newly_blocked and vid not in stopped_vehicles:
                    traci_instance.vehicle.setSpeed(vid, 0)
                    stopped_vehicles.add(vid)
                    print(f"Vehicle {vid} stopped on blocked edge {traci_instance.vehicle.getRoadID(vid)}")
            except Exception:
                continue

        # Reroute affected vehicles
        rerouted = reroute_vehicles(traci_instance, get_successor_edges, get_fuzzy_congestion)
        print(f"Rerouted {rerouted} vehicles after accident.")

    except Exception as e:
        print(f"Error handling accident: {e}")

# -------------------- UTILITIES --------------------
def clear_blocked_edges():
    blocked_edges.clear()
    print("All blocked edges cleared")

def remove_stopped_vehicle(vid):
    if vid in stopped_vehicles:
        stopped_vehicles.remove(vid)
        print(f"Vehicle {vid} removed from stopped vehicles")