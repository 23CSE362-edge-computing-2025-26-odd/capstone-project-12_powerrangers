# config.py
import os

# ThingSpeak Configuration
THINGSPEAK_CHANNEL_ID = os.getenv('THINGSPEAK_CHANNEL_ID', 'YOUR_CHANNEL_ID')
THINGSPEAK_WRITE_API_KEY = os.getenv('THINGSPEAK_WRITE_API_KEY', 'YOUR_WRITE_API_KEY')
THINGSPEAK_READ_API_KEY = os.getenv('THINGSPEAK_READ_API_KEY', 'YOUR_READ_API_KEY')

# Data Configuration
DATA_FIELDS = {
    'field1': 'vehicle_count',
    'field2': 'emergency_detected',
    'field3': 'traffic_density',
    'field4': 'signal_status'
}

# Update interval (seconds)
UPDATE_INTERVAL = 15
