import os

THINGSPEAK_CHANNEL_ID = os.getenv('THINGSPEAK_CHANNEL_ID', '3105957')
THINGSPEAK_WRITE_API_KEY = os.getenv('THINGSPEAK_WRITE_API_KEY', 'J3XLBZGIDAAHFQB6')
THINGSPEAK_READ_API_KEY = os.getenv('THINGSPEAK_READ_API_KEY', 'SPQNWFBRUHEVCZ0T')

DATA_FIELDS = {
    'field1': 'vehicle_count',
    'field2': 'emergency_detected',
    'field3': 'signal_status',
    'field4': 'traffic_density'
}

UPDATE_INTERVAL = 15
