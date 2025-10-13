import streamlit as st
from supabase import create_client, Client
import pandas as pd
import plotly.express as px

# ==== Supabase Setup ====
SUPABASE_URL = "https://viktldenjhwndygewykg.supabase.co"
SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InZpa3RsZGVuamh3bmR5Z2V3eWtnIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTk5MzQ0MjYsImV4cCI6MjA3NTUxMDQyNn0.JYjoyhYFeB6kbWD0vBD_Tmn7-CcpVGz08HgaIt3eM20"
TABLE = "accidents"
# ==== Connect to Supabase ====

# ==== Connect to Supabase ====
supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)

# ==== Page Config ====
st.set_page_config(page_title="SUMO Accident Dashboard", layout="wide")

# ==== Header ====
st.markdown("## 🚦 SUMO Accident Response Dashboard")

# ==== Fetch Data ====
with st.spinner("Loading accident data..."):
    response = supabase.table(TABLE).select("*").execute()
    df = pd.DataFrame(response.data)

if df.empty:
    st.warning("No accident data found in Supabase yet.")
    st.stop()

# ==== Metrics ====
total_accidents = len(df)
avg_response_time = round(df["response_time"].mean(), 2) if "response_time" in df.columns else 0

# Most accident-prone edge
edge_counts = df["accident_edge"].value_counts().reset_index()
edge_counts.columns = ["Edge", "Accidents"]
most_prone_edge = edge_counts.iloc[0]["Edge"] if not edge_counts.empty else "N/A"

# ==== Top Metrics Display ====
c1, c2, c3 = st.columns(3)
with c1:
    st.metric("Total Accidents Handled", total_accidents)
with c2:
    st.metric("Average Ambulance Response Time (s)", avg_response_time)
with c3:
    st.metric("Most Accident-Prone Edge", most_prone_edge)

st.markdown("---")

# ==== Accidents per Edge Bar Chart ====
st.subheader("📊 Accidents per Edge")
fig_edges = px.bar(
    edge_counts,
    x="Edge", y="Accidents",
    text="Accidents",
    color="Edge",
    color_discrete_sequence=px.colors.qualitative.Vivid,
    title="Accident Count by Edge"
)
fig_edges.update_traces(textposition="outside")
fig_edges.update_layout(template="plotly_white")
st.plotly_chart(fig_edges, use_container_width=True)

# ==== Accident Status Pie Chart ====
st.subheader("🟢 Accident Status Distribution")
if "status" in df.columns:
    status_counts = df["status"].value_counts().reset_index()
    status_counts.columns = ["Status", "Count"]
    fig_status = px.pie(
        status_counts, values="Count", names="Status",
        title="Accident Status Distribution", color_discrete_sequence=px.colors.qualitative.Set2
    )
    st.plotly_chart(fig_status, use_container_width=True)

# ==== Response Time Histogram ====
st.subheader("⏱️ Response Time Distribution")
if "response_time" in df.columns:
    fig_response = px.histogram(df, x="response_time", nbins=20, title="Ambulance Response Time Distribution")
    fig_response.update_layout(template="plotly_white", xaxis_title="Response Time (s)", yaxis_title="Count")
    st.plotly_chart(fig_response, use_container_width=True)

# ==== Accident Timeline (per day) ====
st.subheader("📅 Accident Timeline")
if "created_at" in df.columns:
    df["date"] = pd.to_datetime(df["created_at"]).dt.date
    timeline = df.groupby("date").size().reset_index(name="Accidents")
    fig_timeline = px.line(timeline, x="date", y="Accidents", markers=True, title="Accidents Over Time")
    fig_timeline.update_layout(template="plotly_white", xaxis_title="Date", yaxis_title="Number of Accidents")
    st.plotly_chart(fig_timeline, use_container_width=True)

# ==== Accident Heatmap (2D map) ====
st.subheader("🗺️ Accident Locations (Simulation Coordinates)")
if "location_x" in df.columns and "location_y" in df.columns:
    fig_map = px.scatter(
        df, x="location_x", y="location_y",
        color="accident_edge",
        hover_data=["vehicles_involved", "response_time", "status", "created_at"],
        title="Accident Locations on Simulation Map",
        color_discrete_sequence=px.colors.qualitative.Safe
    )
    fig_map.update_layout(template="plotly_white", xaxis_title="X Position (m)", yaxis_title="Y Position (m)", height=600)
    st.plotly_chart(fig_map, use_container_width=True)

# ==== Accident Data Table ====
st.subheader("📋 Accident Records")
display_columns = ["accident_id", "accident_edge", "vehicles_involved", "response_time", "status", "created_at"]
st.dataframe(df[display_columns], use_container_width=True, height=400)

st.markdown("<center>© 2025 Smart Transport Analytics | Powered by SUMO + Supabase</center>", unsafe_allow_html=True)