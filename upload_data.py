# upload_data.py
import requests
import random
import time
from config import THINGSPEAK_CHANNEL_ID, THINGSPEAK_WRITE_API_KEY, UPDATE_INTERVAL

from datetime import datetime

def generate_sample_data():
    vehicle_count = random.randint(5, 50)                    # Number of vehicles
    emergency_detected = random.choice([0, 1])               # 0 = No, 1 = Yes
    vehicle_number = f"AP{random.randint(10,99)}XYZ{random.randint(1000,9999)}"
    accident_type = random.choice(["none", "collision", "rollover", "pedestrian"])
    accident_location = random.choice(["Junction 21", "Main St", "Highway Exit", "Intersection 14"])
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    return {
        'field1': vehicle_count,
        'field2': emergency_detected,
        'field3': vehicle_number,
        'field4': accident_type,
        'field5': accident_location,
        'field6': timestamp
    }

def upload_to_thingspeak(data):
    """Upload data to ThingSpeak"""
    url = f'https://api.thingspeak.com/update?api_key={THINGSPEAK_WRITE_API_KEY}'
    
    # Add data fields to URL
    for field, value in data.items():
        url += f'&{field}={value}'
    
    try:
        response = requests.get(url)
        if response.status_code == 200:
            print(f"Data uploaded successfully: {data}")
            return True
        else:
            print(f"Failed to upload data. Status: {response.status_code}")
            return False
    except Exception as e:
        print(f"Error uploading data: {e}")
        return False

if __name__ == "__main__":
    print("Starting data upload to ThingSpeak...")
    print(f"Channel ID: {THINGSPEAK_CHANNEL_ID}")
    print(f"Update interval: {UPDATE_INTERVAL} seconds\n")
    
    while True:
        data = generate_sample_data()
        upload_to_thingspeak(data)
        time.sleep(UPDATE_INTERVAL)
