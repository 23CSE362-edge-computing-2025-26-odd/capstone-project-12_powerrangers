"""
This generates ALL possible vehicle routes using actual SUMO edges.

Based on edges.xml with 24 bidirectional edges in a 3x3 grid.
Generates comprehensive route combinations for realistic traffic simulation.
"""

import xml.etree.ElementTree as ET

# ----------------------------
# Define ALL SUMO edge connectivity (from edges.xml)
# Each edge maps to its possible next edges
# ----------------------------

edge_connections = {
    # Horizontal edges (left to right)
    "A_B": ["B_C", "B_E", "B_A"],
    "B_C": ["C_F", "C_B"],
    "D_E": ["E_F", "E_H", "E_B", "E_D"],
    "E_F": ["F_I", "F_C", "F_E"],
    "G_H": ["H_I", "H_E", "H_G"],
    "H_I": ["I_F", "I_H"],
    
    # Horizontal edges (right to left)
    "B_A": ["A_D", "A_B"],
    "C_B": ["B_A", "B_E", "B_C"],
    "E_D": ["D_A", "D_G", "D_E"],
    "F_E": ["E_B", "E_D", "E_H", "E_F"],
    "H_G": ["G_D", "G_H"],
    "I_H": ["H_G", "H_E", "H_I"],
    
    # Vertical edges (top to bottom)
    "A_D": ["D_E", "D_G", "D_A"],
    "D_G": ["G_H", "G_D"],
    "B_E": ["E_D", "E_F", "E_H", "E_B"],
    "E_H": ["H_G", "H_I", "H_E"],
    "C_F": ["F_E", "F_I", "F_C"],
    "F_I": ["I_H", "I_F"],
    
    # Vertical edges (bottom to top)
    "D_A": ["A_B", "A_D"],
    "G_D": ["D_A", "D_E", "D_G"],
    "E_B": ["B_A", "B_C", "B_E"],
    "H_E": ["E_B", "E_D", "E_F", "E_H"],
    "F_C": ["C_B", "C_F"],
    "I_F": ["F_C", "F_E", "F_I"],
}

# All edges in the network
all_edges = list(edge_connections.keys())

# Boundary edges where routes can terminate
boundary_edges = [
    "A_B", "A_D",  # From A
    "B_A", "B_C",  # From B
    "C_B", "C_F",  # From C
    "D_A", "D_G",  # From D
    "F_C", "F_I",  # From F
    "G_D", "G_H",  # From G
    "H_G", "H_I",  # From H
    "I_F", "I_H"   # From I
]

# ----------------------------
# Recursive route finding using SUMO edges
# ----------------------------

def find_routes_from_edge(start_edge, path=None, max_depth=6):
    """
    Recursively find all valid routes starting from start_edge.
    
    Args:
        start_edge: Starting SUMO edge (e.g., "A_B")
        path: Current path (list of edges)
        max_depth: Maximum route length
    
    Returns:
        List of valid routes (each route is a list of edge IDs)
    """
    if path is None:
        path = [start_edge]
    else:
        path = path + [start_edge]
    
    routes = []
    
    # If we reach a boundary edge and path length > 1, save the route
    if start_edge in boundary_edges and len(path) > 1:
        routes.append(path)
    
    # Stop if maximum depth reached
    if len(path) >= max_depth:
        return routes
    
    # Explore connected edges (avoid cycles)
    if start_edge in edge_connections:
        for next_edge in edge_connections[start_edge]:
            if next_edge not in path:  # Avoid revisiting edges
                routes.extend(find_routes_from_edge(next_edge, path, max_depth))
    
    return routes

# ----------------------------
# Generate all possible routes
# ----------------------------

print("Generating routes from all edges...")
all_routes = []

for edge in all_edges:
    routes_from_edge = find_routes_from_edge(edge, max_depth=8)
    all_routes.extend(routes_from_edge)
    print(f"  From {edge}: {len(routes_from_edge)} routes")

# Remove duplicates
unique_routes = []
seen = set()
for route in all_routes:
    route_str = " ".join(route)
    if route_str not in seen:
        seen.add(route_str)
        unique_routes.append(route)

print(f"\nTotal unique routes generated: {len(unique_routes)}")

# ----------------------------
# Generate SUMO XML
# ----------------------------

routes_root = ET.Element("routes")

# Vehicle types
vtypes = [
    {"id": "fastCar", "accel": "4.0", "decel": "6.0", "sigma": "0.5", 
     "length": "5", "maxSpeed": "20.0", "color": "0,1,0", "guiShape": "passenger/sedan"},
    {"id": "slowCar", "accel": "2.0", "decel": "4.0", "sigma": "0.5", 
     "length": "5", "maxSpeed": "10.0", "color": "0,0,1", "guiShape": "passenger/hatchback"},
    {"id": "ambulance", "accel": "3.0", "decel": "6.0", "sigma": "0.5", 
     "length": "10", "maxSpeed": "13.9", "color": "1,1,1", "guiShape": "emergency"}
]

for vt in vtypes:
    ET.SubElement(routes_root, "vType", vt)

# Add all generated routes
for idx, route in enumerate(unique_routes):
    edges_str = " ".join(route)
    ET.SubElement(routes_root, "route", id=f"route{idx}", edges=edges_str)

# ----------------------------
# Add ambulance parking routes
# ----------------------------

ambulance_parking_routes = ["A_B_parking", "D_G_parking", "H_I_parking"]
for i, edge in enumerate(ambulance_parking_routes):
    ET.SubElement(routes_root, "route", id=f"routeAmbulance{i}", edges=edge)

# ----------------------------
# Format and write XML
# ----------------------------

routes_root.text = "\n"
for child in routes_root:
    child.tail = "\n"

tree = ET.ElementTree(routes_root)
tree.write("vehicles.rou.xml", encoding="UTF-8", xml_declaration=True)

print(f"\n✅ Generated {len(unique_routes)} normal routes + {len(ambulance_parking_routes)} ambulance parking routes")
print(f"📄 Saved to vehicles.rou.xml")

# ----------------------------
# Print some example routes
# ----------------------------

print("\n📋 Sample routes:")
for i in range(min(10, len(unique_routes))):
    print(f"  route{i}: {' -> '.join(unique_routes[i])}")
