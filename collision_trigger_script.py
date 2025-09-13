#!/usr/bin/env python3
"""
Manual collision trigger script for testing V2V communication
Run this script while the simulation is running to force collisions
"""

import traci
import time
import sys

def connect_to_simulation():
    """Connect to running SUMO simulation"""
    try:
        traci.init(port=8813)
        print("Connected to SUMO simulation")
        return True
    except Exception as e:
        print(f"Failed to connect to simulation: {e}")
        return False

def force_collision():
    """Force a collision between two vehicles"""
    vehicles = traci.vehicle.getIDList()
    print(f"Available vehicles: {list(vehicles)}")
    
    if len(vehicles) >= 2:
        # Select first two vehicles
        vid1, vid2 = vehicles[0], vehicles[1]
        
        try:
            # Get positions
            pos1 = traci.vehicle.getPosition(vid1)
            pos2 = traci.vehicle.getPosition(vid2)
            
            print(f"Forcing collision between {vid1} and {vid2}")
            print(f"{vid1} position: {pos1}")
            print(f"{vid2} position: {pos2}")
            
            # Stop both vehicles
            traci.vehicle.setSpeed(vid1, 0)
            traci.vehicle.setSpeed(vid2, 0)
            
            # Make them red (accident color)
            traci.vehicle.setColor(vid1, (255, 0, 0, 255))
            traci.vehicle.setColor(vid2, (255, 0, 0, 255))
            
            print(f"Collision forced! Vehicles stopped and marked red.")
            
        except Exception as e:
            print(f"Error forcing collision: {e}")
    else:
        print("Need at least 2 vehicles to force collision")

def main():
    if not connect_to_simulation():
        sys.exit(1)
    
    print("Manual Collision Trigger")
    print("Press 'c' to force collision, 'q' to quit")
    
    try:
        while True:
            cmd = input("> ").strip().lower()
            
            if cmd == 'c':
                force_collision()
            elif cmd == 'q':
                break
            elif cmd == 'status':
                vehicles = traci.vehicle.getIDList()
                print(f"Current vehicles: {len(vehicles)}")
                for vid in vehicles:
                    try:
                        pos = traci.vehicle.getPosition(vid)
                        speed = traci.vehicle.getSpeed(vid)
                        print(f"  {vid}: pos=({pos[0]:.1f}, {pos[1]:.1f}), speed={speed:.1f}m/s")
                    except:
                        print(f"  {vid}: data unavailable")
            else:
                print("Commands: 'c' (collision), 'status' (show vehicles), 'q' (quit)")
    
    except KeyboardInterrupt:
        pass
    finally:
        traci.close()
        print("Disconnected from simulation")

if __name__ == "__main__":
    main()
