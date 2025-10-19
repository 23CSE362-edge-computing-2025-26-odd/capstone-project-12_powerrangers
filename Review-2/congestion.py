import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import traci

# -------------------- Fuzzy Logic Setup --------------------
vehicle_count = ctrl.Antecedent(np.arange(0, 5, 1), 'vehicle_count')
congestion = ctrl.Consequent(np.arange(0, 11, 1), 'congestion')

# Vehicle categories with slight overlap to avoid gaps
vehicle_count['zero'] = fuzz.trimf(vehicle_count.universe, [0, 0, 0.5])
vehicle_count['one'] = fuzz.trimf(vehicle_count.universe, [0.5, 1, 1.5])
vehicle_count['two'] = fuzz.trimf(vehicle_count.universe, [1.5, 2, 2.5])
vehicle_count['three_plus'] = fuzz.trimf(vehicle_count.universe, [2.5, 4, 4])

# Congestion levels with overlap
congestion['low'] = fuzz.trimf(congestion.universe, [0, 0, 3])
congestion['medium'] = fuzz.trimf(congestion.universe, [2, 4, 6])
congestion['high'] = fuzz.trimf(congestion.universe, [5, 8, 10])
congestion['critical'] = fuzz.trimf(congestion.universe, [8, 10, 10])

# Rules
rule0 = ctrl.Rule(vehicle_count['zero'], congestion['low'])
rule1 = ctrl.Rule(vehicle_count['one'], congestion['medium'])
rule2 = ctrl.Rule(vehicle_count['two'], congestion['high'])
rule3 = ctrl.Rule(vehicle_count['three_plus'], congestion['critical'])

# Create control system
congestion_ctrl = ctrl.ControlSystem([rule0, rule1, rule2, rule3])

def get_fuzzy_congestion(edge):
    """
    Returns congestion level (0-10) based on vehicle count
    """
    # Create fresh simulation each time
    sim = ctrl.ControlSystemSimulation(congestion_ctrl)
    
    try:
        # Get vehicle count from SUMO
        veh_count = len(traci.edge.getLastStepVehicleIDs(edge))
        veh_count = float(min(veh_count, 4))
        
        # Compute fuzzy output
        sim.input['vehicle_count'] = veh_count
        sim.compute()
        
        return sim.output['congestion']
    
    except Exception as e:
        # Fallback: simple linear mapping
        print(f"Fuzzy error for edge {edge}: {e}")
        veh_count = len(traci.edge.getLastStepVehicleIDs(edge))
        return min(veh_count * 2.5, 10.0)