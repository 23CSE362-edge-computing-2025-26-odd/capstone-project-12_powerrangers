import xml.etree.ElementTree as ET

# ----------------------------
nodes = ["A","B","C","D","E","F","G","H","I"]

edges = {
    "A": ["B", "D"],
    "B": ["A", "C", "E"],
    "C": ["B", "F"],
    "D": ["A", "E", "G"],
    "E": ["B", "D", "F", "H"],
    "F": ["C", "E", "I"],
    "G": ["D", "H"],
    "H": ["G", "E", "I"],
    "I": ["F", "H"]
}

boundary_nodes = ["A", "B", "C", "D", "F", "G", "H", "I"]

def find_routes(start, path=None, max_depth=6):
    if path is None:
        path = [start]
    else:
        path = path + [start]

    routes = []
    if start in boundary_nodes and len(path) > 1:
        routes.append(path)
    if len(path) >= max_depth:
        return routes
    for neighbor in edges[start]:
        if neighbor not in path:
            routes.extend(find_routes(neighbor, path, max_depth))
    return routes

# Generate all normal routes
all_routes = []
for node in nodes:
    all_routes.extend(find_routes(node))

# ----------------------------
# SUMO XML generation
routes_root = ET.Element("routes")

vtypes = [
    {"id": "fastCar", "accel":"4.0","decel":"6.0","sigma":"0.5","length":"5","maxSpeed":"20.0","color":"0,1,0","guiShape":"passenger/sedan"},
    {"id": "slowCar", "accel":"2.0","decel":"4.0","sigma":"0.5","length":"5","maxSpeed":"10.0","color":"0,0,1","guiShape":"passenger/hatchback"},
    {"id": "ambulance", "accel":"3.0","decel":"6.0","sigma":"0.5","length":"10","maxSpeed":"13.9","color":"1,1,1","guiShape":"emergency"}
]

for vt in vtypes:
    ET.SubElement(routes_root, "vType", vt)

# Normal routes
for idx, route in enumerate(all_routes):
    edges_str = " ".join([f"{route[i]}_{route[i+1]}" for i in range(len(route)-1)])
    ET.SubElement(routes_root, "route", id=f"route{idx}", edges=edges_str)

# ----------------------------
# Hardcoded ambulance parking routes
ambulance_parking_routes = ["A_B_parking", "D_G_parking", "H_I_parking"]
for i, edge in enumerate(ambulance_parking_routes):
    ET.SubElement(routes_root, "route", id=f"routeAmbulance{i}", edges=edge)

# Pretty-print the XML with newlines
routes_root.text = "\n"
for child in routes_root:
    child.tail = "\n"

tree = ET.ElementTree(routes_root)
tree.write("vehicles.rou.xml", encoding="UTF-8", xml_declaration=True)

print(f"Generated {len(all_routes)} normal routes + {len(ambulance_parking_routes)} ambulance parking routes")
