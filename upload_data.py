# upload_data.py
import requests
import random
import time
from config import THINGSPEAK_CHANNEL_ID, THINGSPEAK_WRITE_API_KEY, UPDATE_INTERVAL

def generate_sample_data():
    """Generate sample sensor data"""
    return {
        'field1': round(random.uniform(20.0, 30.0), 2),  # Temperature
        'field2': round(random.uniform(40.0, 60.0), 2),  # Humidity
        'field3': round(random.uniform(980.0, 1020.0), 2),  # Pressure
        'field4': round(random.uniform(50.0, 150.0), 2)  # Air Quality
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
