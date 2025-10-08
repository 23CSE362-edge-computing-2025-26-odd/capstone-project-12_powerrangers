# dashboard.py
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from fetch_data import fetch_data
from config import DATA_FIELDS, UPDATE_INTERVAL
import pandas as pd

# Initialize the Dash app
app = dash.Dash(__name__)

# Define the layout
app.layout = html.Div([
    html.H1("IoT Sensor Data Dashboard", style={'textAlign': 'center'}),
    
    html.Div([
        html.H3("Real-time Sensor Monitoring"),
        dcc.Interval(
            id='interval-component',
            interval=UPDATE_INTERVAL*1000,  # milliseconds
            n_intervals=0
        )
    ]),
    
    html.Div([
        html.Div([
            dcc.Graph(id='temperature-graph')
        ], className='six columns'),
        
        html.Div([
            dcc.Graph(id='humidity-graph')
        ], className='six columns'),
    ], className='row'),
    
    html.Div([
        html.Div([
            dcc.Graph(id='pressure-graph')
        ], className='six columns'),
        
        html.Div([
            dcc.Graph(id='air-quality-graph')
        ], className='six columns'),
    ], className='row'),
    
    html.Div([
        html.H3("Latest Readings"),
        html.Div(id='latest-readings')
    ], style={'margin': '20px'})
])

def create_graph(df, field_name, title, color):
    """Create a plotly graph for a sensor field"""
    if df is None or field_name not in df.columns:
        return go.Figure()
    
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df['created_at'],
        y=df[field_name],
        mode='lines+markers',
        name=field_name,
        line=dict(color=color, width=2),
        marker=dict(size=6)
    ))
    
    fig.update_layout(
        title=title,
        xaxis_title='Time',
        yaxis_title='Value',
        hovermode='x unified'
    )
    
    return fig

@app.callback(
    [Output('temperature-graph', 'figure'),
     Output('humidity-graph', 'figure'),
     Output('pressure-graph', 'figure'),
     Output('air-quality-graph', 'figure'),
     Output('latest-readings', 'children')],
    [Input('interval-component', 'n_intervals')]
)
def update_graphs(n):
    """Update all graphs with latest data"""
    df = fetch_data(results=100)
    
    if df is None:
        return [{}, {}, {}, {}, "No data available"]
    
    # Create graphs
    temp_fig = create_graph(df, 'temperature', 'Temperature (°C)', 'red')
    humidity_fig = create_graph(df, 'humidity', 'Humidity (%)', 'blue')
    pressure_fig = create_graph(df, 'pressure', 'Pressure (hPa)', 'green')
    air_quality_fig = create_graph(df, 'air_quality', 'Air Quality Index', 'orange')
    
    # Create latest readings display
    if len(df) > 0:
        latest = df.iloc[-1]
        readings_div = html.Div([
            html.P(f"Temperature: {latest.get('temperature', 'N/A'):.2f} °C"),
            html.P(f"Humidity: {latest.get('humidity', 'N/A'):.2f} %"),
            html.P(f"Pressure: {latest.get('pressure', 'N/A'):.2f} hPa"),
            html.P(f"Air Quality: {latest.get('air_quality', 'N/A'):.2f}"),
            html.P(f"Last Updated: {latest['created_at']}", style={'fontStyle': 'italic'})
        ])
    else:
        readings_div = "No data available"
    
    return temp_fig, humidity_fig, pressure_fig, air_quality_fig, readings_div

if __name__ == '__main__':
    print("Starting dashboard server...")
    print("Open http://localhost:8050 in your browser")
    app.run_server(debug=True, port=8050)
