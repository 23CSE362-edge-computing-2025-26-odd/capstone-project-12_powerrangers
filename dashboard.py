# dashboard.py
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from fetch_data import fetch_data
from config import UPDATE_INTERVAL
import pandas as pd

app = dash.Dash(__name__)

app.layout = html.Div([
    html.H1("Traffic Accident & Emergency Dashboard", style={'textAlign': 'center'}),

    html.Div([
        html.H3("Real-time Accident Event Monitoring"),
        dcc.Interval(
            id='interval-component',
            interval=UPDATE_INTERVAL*1000,
            n_intervals=0
        )
    ]),

    html.Div([
        dcc.Graph(id='vehicle-count-graph')
    ]),

    html.Div([
        html.H3("Latest Logged Accident Event"),
        html.Div(id='latest-event')
    ], style={'margin': '20px'}),

    html.Div([
        html.H3("Recent Events Table"),
        html.Div(id='events-table')
    ], style={'margin': '20px'})
])

def create_line_graph(df, y_field, title, color):
    if df is None or y_field not in df.columns:
        return go.Figure()

    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df['created_at'],
        y=df[y_field],
        mode='lines+markers',
        name=y_field,
        line=dict(color=color, width=2),
        marker=dict(size=6)
    ))

    fig.update_layout(
        title=title,
        xaxis_title='Time',
        yaxis_title=y_field.replace('_', ' ').title(),
        hovermode='x unified'
    )

    return fig

@app.callback(
    [Output('vehicle-count-graph', 'figure'),
     Output('latest-event', 'children'),
     Output('events-table', 'children')],
    [Input('interval-component', 'n_intervals')]
)
def update_dashboard(n):
    df = fetch_data(results=50)

    # Vehicle Count chart
    vehicle_fig = create_line_graph(df, 'vehicle_count', 'Vehicle Count Over Time', 'blue')

    # Latest Accident Event
    if df is not None and len(df) > 0:
        latest = df.iloc[-1]
        latest_div = html.Ul([
            html.Li(f"Accident Time: {latest.get('timestamp', 'N/A')}"),
            html.Li(f"Location: {latest.get('accident_location', 'N/A')}"),
            html.Li(f"Accident Type: {latest.get('accident_type', 'N/A')}"),
            html.Li(f"Emergency Detected: {int(latest.get('emergency_detected', 0))}"),
            html.Li(f"Vehicle Number: {latest.get('accident_vehicle_number', 'N/A')}"),
            html.Li(f"Reported at: {latest.get('created_at')}")
        ])
        # Table of recent events
        table_df = df[['created_at','accident_vehicle_number','accident_type','accident_location','vehicle_count','emergency_detected']].tail(10)
        table = html.Table([
            html.Tr([html.Th(col) for col in table_df.columns])] +
            [html.Tr([html.Td(table_df.iloc[i][col]) for col in table_df.columns]) for i in range(table_df.shape[0])]
        )
    else:
        latest_div = "No data available"
        table = "No data"

    return vehicle_fig, latest_div, table

if __name__ == '__main__':
    print("Starting dashboard server...")
    print("Open http://localhost:8050 in your browser")
    app.run_server(debug=True, port=8050)
