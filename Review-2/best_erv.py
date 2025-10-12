import random
import math
import heapq
import traci
from congestion import get_fuzzy_congestion

# -------------------- GA Parameters --------------------
POP_SIZE = 20
GENERATIONS = 30
TOURNAMENT_K = 3
CROSSOVER_RATE = 0.8
MUTATION_RATE = 0.1

# -------------------- Ambulance Data --------------------
ambulance_readiness = {
    "ambulance0": 85,
    "ambulance1": 70,
    "ambulance2": 90
}

ambulance_routes = {
    "ambulance0": "routeAmbulance0",
    "ambulance1": "routeAmbulance1",
    "ambulance2": "routeAmbulance2"
}

ambulance_anchor_edges = {
    "ambulance0": "A_B",
    "ambulance1": "D_G",
    "ambulance2": "H_I"
}

# Cache for computed routes to avoid recalculation
route_cache = {}

# =======================================================
# ROUTE FINDING FUNCTIONS
# =======================================================
def get_edge_weight(edge_id):
    """Calculate edge weight based on length and congestion"""
    try:
        edge_length = traci.edge.getLength(edge_id)
        try:
            congestion = get_fuzzy_congestion(edge_id)
        except:
            congestion = 5.0
        congestion_factor = congestion / 10.0
        weight = edge_length * (1 + congestion_factor * 2)
        return weight
    except:
        return 100.0

def get_edge_neighbors(edge_id):
    """Get all edges reachable from this edge"""
    neighbors = []
    try:
        lane_id = edge_id + "_0"
        links = traci.lane.getLinks(lane_id)
        for link in links:
            next_lane = link[0]
            if next_lane:
                next_edge = next_lane.rsplit('_', 1)[0]
                if not next_edge.startswith(':') and 'parking' not in next_edge.lower():
                    neighbors.append(next_edge)
    except:
        pass
    return neighbors

def dijkstra_shortest_path(start_edge, end_edge):
    """Find optimal path considering distance and congestion"""
    if start_edge == end_edge:
        return [start_edge], 0
    
    # Check cache
    cache_key = (start_edge, end_edge)
    if cache_key in route_cache:
        return route_cache[cache_key]
    
    pq = [(0, start_edge, [start_edge])]
    visited = set()
    costs = {start_edge: 0}
    
    max_iterations = 300
    iterations = 0
    
    while pq and iterations < max_iterations:
        iterations += 1
        current_cost, current_edge, path = heapq.heappop(pq)
        
        if current_edge in visited:
            continue
        
        visited.add(current_edge)
        
        if current_edge == end_edge:
            route_cache[cache_key] = (path, current_cost)
            return path, current_cost
        
        neighbors = get_edge_neighbors(current_edge)
        
        for next_edge in neighbors:
            if next_edge in visited:
                continue
            
            edge_weight = get_edge_weight(next_edge)
            new_cost = current_cost + edge_weight
            
            if next_edge not in costs or new_cost < costs[next_edge]:
                costs[next_edge] = new_cost
                new_path = path + [next_edge]
                heapq.heappush(pq, (new_cost, next_edge, new_path))
    
    # Fallback to SUMO routing
    try:
        route_result = traci.simulation.findRoute(start_edge, end_edge)
        if route_result and route_result.edges:
            fallback_route = list(route_result.edges)
            fallback_cost = sum(get_edge_weight(e) for e in fallback_route)
            route_cache[cache_key] = (fallback_route, fallback_cost)
            return fallback_route, fallback_cost
    except:
        pass
    
    return None, float('inf')

# =======================================================
# FITNESS FUNCTION WITH ROUTE OPTIMIZATION
# =======================================================
def fitness(individual, accident_x, accident_y, positions, accident_edge=None):
    """
    Fitness considers:
    1. Ambulance readiness
    2. Straight-line distance (initial estimate)
    3. Actual route cost (distance + congestion)
    4. Edge proximity bonus
    """
    if individual not in positions:
        return -9999.0

    readiness = ambulance_readiness.get(individual, 50)
    
    # Straight-line distance (quick estimate)
    (x, y) = positions[individual]
    straight_dist = math.sqrt((x - accident_x) ** 2 + (y - accident_y) ** 2)
    
    # Get ambulance current edge and find optimal route
    try:
        current_edge = traci.vehicle.getRoadID(individual)
        if current_edge and accident_edge and not current_edge.startswith(':'):
            # Find optimal route with congestion consideration
            optimal_route, route_cost = dijkstra_shortest_path(current_edge, accident_edge)
            
            if optimal_route:
                # Use actual route cost
                route_distance = route_cost
            else:
                # Fallback to straight-line distance
                route_distance = straight_dist
        else:
            route_distance = straight_dist
    except:
        route_distance = straight_dist
    
    # Normalize components
    readiness_norm = readiness / 100.0
    distance_penalty = route_distance / 500.0  # Normalize by typical max distance
    
    # Edge proximity bonus
    edge_bonus = 0.0
    if accident_edge:
        anchor_edge = ambulance_anchor_edges.get(individual)
        if anchor_edge == accident_edge:
            edge_bonus = 1.5  # Strong bonus for same edge
        elif anchor_edge and accident_edge:
            # Check if they share nodes
            if (anchor_edge[0] == accident_edge[0]) or (anchor_edge[-1] == accident_edge[-1]):
                edge_bonus = 0.8
    
    # Final fitness score
    score = readiness_norm - distance_penalty + edge_bonus
    
    return score

# =======================================================
# SELECTION / CROSSOVER / MUTATION
# =======================================================
def tournament_selection(pop, fitnesses):
    best = random.choice(pop)
    for _ in range(TOURNAMENT_K - 1):
        challenger = random.choice(pop)
        if fitnesses[challenger] > fitnesses[best]:
            best = challenger
    return best

def crossover(parent1, parent2):
    if random.random() < CROSSOVER_RATE:
        return random.choice([parent1, parent2])
    return parent1

def mutate(individual):
    if random.random() < MUTATION_RATE:
        return random.choice(list(ambulance_readiness.keys()))
    return individual

# =======================================================
# GA DRIVER WITH ROUTE COMPUTATION
# =======================================================
def select_best_ambulance(accident_x, accident_y, positions, accident_edge):
    """
    Select best ambulance using GA that considers:
    - Readiness
    - Optimal route (distance + congestion)
    - Edge proximity
    
    Returns:
        tuple: (best_ambulance_id, optimal_route)
    """
    # Clear route cache for fresh computation
    route_cache.clear()
    
    population = [random.choice(list(ambulance_readiness.keys())) for _ in range(POP_SIZE)]

    for gen in range(GENERATIONS):
        fitnesses = {ind: fitness(ind, accident_x, accident_y, positions, accident_edge)
                     for ind in population}
        new_population = []
        while len(new_population) < POP_SIZE:
            p1 = tournament_selection(population, fitnesses)
            p2 = tournament_selection(population, fitnesses)
            child = crossover(p1, p2)
            child = mutate(child)
            new_population.append(child)
        population = new_population

    # Final selection
    final_fitnesses = {ind: fitness(ind, accident_x, accident_y, positions, accident_edge)
                       for ind in population}
    best = max(final_fitnesses, key=final_fitnesses.get)
    
    # Get the optimal route for the best ambulance
    optimal_route = None
    try:
        current_edge = traci.vehicle.getRoadID(best)
        if current_edge and accident_edge and not current_edge.startswith(':'):
            optimal_route, route_cost = dijkstra_shortest_path(current_edge, accident_edge)
            print(f"[GA] Selected: {best}")
            print(f"[GA] Fitness: {final_fitnesses[best]:.3f}")
            print(f"[GA] Route cost: {route_cost:.2f}")
            if optimal_route:
                print(f"[GA] Route: {' -> '.join(optimal_route[:5])}{'...' if len(optimal_route) > 5 else ''}")
    except Exception as e:
        print(f"[GA] Route computation error: {e}")
    
    return best, optimal_route