# IoT Data Analytics with ThingSpeak

## Overview
This project provides a complete solution for collecting, storing, and visualizing IoT sensor data using ThingSpeak cloud platform.

## Features
- Real-time data collection from IoT sensors
- Cloud storage using ThingSpeak
- Interactive dashboard with live graphs
- Support for multiple sensor types (temperature, humidity, pressure, air quality)

## Setup Instructions

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Configure ThingSpeak
1. Create a ThingSpeak account at https://thingspeak.com
2. Create a new channel
3. Add the following fields:
   - Field 1: Temperature
   - Field 2: Humidity
   - Field 3: Pressure
   - Field 4: Air Quality
4. Copy your Channel ID and API Keys

### 3. Set Environment Variables
Create a `.env` file with:
```
THINGSPEAK_CHANNEL_ID=your_channel_id
THINGSPEAK_WRITE_API_KEY=your_write_api_key
THINGSPEAK_READ_API_KEY=your_read_api_key
```

## Usage

### Upload Data
```bash
python upload_data.py
```

### Fetch Data
```bash
python fetch_data.py
```

### Run Dashboard
```bash
python dashboard.py
```
Then open http://localhost:8050 in your browser

## Project Structure
- `config.py` - Configuration settings
- `upload_data.py` - Upload sensor data to ThingSpeak
- `fetch_data.py` - Retrieve data from ThingSpeak
- `dashboard.py` - Interactive visualization dashboard
- `requirements.txt` - Python dependencies

## License
MIT License
