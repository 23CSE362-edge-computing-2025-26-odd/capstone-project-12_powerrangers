import time
import random

class CENBroadcast:
    def __init__(self, interval=300):
        """
        interval: broadcast interval in seconds (default 300s = 5 minutes)
        """
        self.interval = interval
        self.accidents = {}
    
    def register(self, accident_id, location, sim_time):
        """Register a new accident for broadcasting."""
        self.accidents[accident_id] = {
            "location": location,
            "last_time": sim_time
        }
        print(f" Accident {accident_id} registered at {location}, starting broadcasts.")
    
    def broadcast(self, sim_time):
        """Call this every simulation step. Broadcasts messages at interval."""
        for acc_id, data in self.accidents.items():
            if sim_time - data["last_time"] >= self.interval:
                print(f" [CEN] Accident at {data['location']} (id={acc_id}) - time={sim_time:.1f}s")
                data["last_time"] = sim_time
