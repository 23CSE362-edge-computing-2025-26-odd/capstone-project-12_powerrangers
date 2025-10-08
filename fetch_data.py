# fetch_data.py
import requests
import pandas as pd
from config import THINGSPEAK_CHANNEL_ID, THINGSPEAK_READ_API_KEY, DATA_FIELDS

def fetch_data(results=100):
    """Fetch data from ThingSpeak
    
    Args:
        results: Number of data points to retrieve (default: 100)
    
    Returns:
        pandas DataFrame with sensor data
    """
    url = f'https://api.thingspeak.com/channels/{THINGSPEAK_CHANNEL_ID}/feeds.json'
    params = {
        'api_key': THINGSPEAK_READ_API_KEY,
        'results': results
    }
    
    try:
        response = requests.get(url, params=params)
        
        if response.status_code == 200:
            data = response.json()
            feeds = data.get('feeds', [])
            
            if not feeds:
                print("No data available")
                return None
            
            # Convert to DataFrame
            df = pd.DataFrame(feeds)
            
            # Rename columns based on DATA_FIELDS
            column_mapping = {field: name for field, name in DATA_FIELDS.items()}
            df = df.rename(columns=column_mapping)
            
            # Convert timestamp to datetime
            df['created_at'] = pd.to_datetime(df['created_at'])
            
            # Convert numeric fields
            for field_name in DATA_FIELDS.values():
                if field_name in df.columns:
                    df[field_name] = pd.to_numeric(df[field_name], errors='coerce')
            
            print(f"Successfully fetched {len(df)} data points")
            return df
        else:
            print(f"Error fetching data. Status: {response.status_code}")
            return None
            
    except Exception as e:
        print(f"Error: {e}")
        return None

if __name__ == "__main__":
    print(f"Fetching data from ThingSpeak Channel: {THINGSPEAK_CHANNEL_ID}\n")
    
    # Fetch last 100 data points
    df = fetch_data(results=100)
    
    if df is not None:
        print("\nData Summary:")
        print(df.describe())
        print("\nLatest 5 entries:")
        print(df.tail())
