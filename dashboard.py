"""
DTS Cable Temperature Monitoring - Operator Dashboard
=====================================================
Provides the two operator-facing screens from the reference
infographic:

  1. Local HMI at Station A - detailed temperature profile
  2. Regional Control Center (SCADA) - route overview and status

Alarm states are read FROM the PLC via plc_bridge, not computed here.
The dashboard is a display layer only, which is how a real HMI works.

Run with:
    streamlit run dashboard.py
"""

import datetime
import io

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st

from thermal_model import (
    CABLE, ROUTE_LENGTH_KM, JOINT_LOCATIONS_KM,
    RouteConditions, find_hotspot, rating_margin,
)
from plc_bridge import PlcLink, evaluate_once

st.set_page_config(page_title="380 kV DTS Monitoring", layout="wide")

# --- Persistent state -----------------------------------------------
if "event_log" not in st.session_state:
    st.session_state.event_log = []
if "link" not in st.session_state:
    st.session_state.link = PlcLink()
    st.session_state.link.connect()

link = st.session_state.link

STATE_COLORS = {
    0: ("Normal", "#1D9E75"),
    1: ("Warning", "#EF9F27"),
    2: ("Alarm", "#E24B4A"),
    3: ("Critical", "#A32D2D"),
    4: ("Comms failure", "#888780"),
}


# =====================================================================
# Sidebar - operating conditions
# =====================================================================
st.sidebar.header("Operating conditions")

load_factor = st.sidebar.slider(
    "Load factor", 0.50, 1.20, 1.00, 0.01,
    help=f"Circuit current as a fraction of the {CABLE.rated_current:.0f} A rated current. "
         f"1.00 = {CABLE.rated_current:.0f} A."
)
ambient_c = st.sidebar.slider(
    "Ambient soil temperature (C)", 15.0, 45.0, 25.0, 1.0,
    help="Undisturbed ground temperature at burial depth, away from the cable. "
         "Conductor temperature is this value plus the computed temperature rise."
)

st.sidebar.subheader("Route condition")
hotspot_enabled = st.sidebar.toggle("Hotspot present", value=True)
hotspot_km = st.sidebar.slider("Hotspot location (km)", 0.0, ROUTE_LENGTH_KM, 42.3, 0.5)
hotspot_rho = st.sidebar.slider(
    "Hotspot soil resistivity (K.m/W)", 1.0, 3.0, 1.8, 0.05,
    help="Normal moist soil is 1.0. Dried-out soil rises well above this."
)

st.sidebar.divider()
if link.is_connected:
    st.sidebar.success("OpenPLC connected")
else:
    st.sidebar.warning("OpenPLC not connected - using fallback logic")
    if st.sidebar.button("Retry PLC connection"):
        link.connect()
        st.rerun()

# =====================================================================
# Run one measurement cycle
# =====================================================================
cond = RouteConditions(
    ambient_c=ambient_c,
    hotspot_km=hotspot_km if hotspot_enabled else None,
    hotspot_soil_rho=hotspot_rho,
)

distance, t_cond, t_sheath, rho, hs_km, peak_c, status = evaluate_once(
    load_factor=load_factor, cond=cond, link=link
)
margin = rating_margin(t_cond)
state_name, state_color = STATE_COLORS.get(status.alarm_state, ("Unknown", "#888780"))

# Log state transitions only, not every rerun
last = st.session_state.event_log[-1] if st.session_state.event_log else None
if last is None or last["state"] != state_name:
    st.session_state.event_log.append({
        "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "state": state_name,
        "hotspot_km": round(hs_km, 1),
        "peak_conductor_C": round(peak_c, 1),
        "load_pct": int(load_factor * 100),
        "margin_K": round(margin, 1),
    })

# =====================================================================
# Screens
# =====================================================================
tab_hmi, tab_scada, tab_log = st.tabs([
    "Local HMI (Station A)",
    "Regional Control Center",
    "Event log",
])

# ---------------------------------------------------------------- HMI
with tab_hmi:
    st.subheader("Cable temperature profile")
    st.caption("380 kV XLPE route, Station A to Station B")

    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=distance, y=t_cond, mode="lines", name="Conductor",
        line=dict(color="#D85A30", width=2),
    ))
    fig.add_trace(go.Scatter(
        x=distance, y=t_sheath, mode="lines", name="Outer sheath",
        line=dict(color="#378ADD", width=2, dash="dot"),
    ))
    fig.add_hline(y=CABLE.T_alarm, line_dash="dash", line_color="#E24B4A",
                  annotation_text=f"Alarm {CABLE.T_alarm:.0f} C")
    fig.add_hline(y=CABLE.T_max_xlpe, line_dash="dash", line_color="#A32D2D",
                  annotation_text=f"XLPE limit {CABLE.T_max_xlpe:.0f} C")
    for j in JOINT_LOCATIONS_KM:
        fig.add_vline(x=j, line_dash="dot", line_color="rgba(136,135,128,0.4)")
    fig.add_trace(go.Scatter(
        x=[hs_km], y=[peak_c], mode="markers+text",
        text=[f"{peak_c:.1f} C"], textposition="top center",
        marker=dict(color="#E24B4A", size=13, symbol="diamond"),
        name="Hotspot",
    ))
    fig.update_layout(
        xaxis_title="Distance from Station A (km)",
        yaxis_title="Temperature (C)",
        height=420, margin=dict(t=30, b=10),
        legend=dict(orientation="h", y=1.12),
    )
    st.plotly_chart(fig, use_container_width=True)

    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Hotspot temperature", f"{peak_c:.1f} C", f"at {hs_km:.1f} km")
    c2.metric("Sheath at hotspot", f"{np.max(t_sheath):.1f} C")
    c3.metric("Margin to 90 C limit", f"{margin:.1f} K",
              delta=f"{margin:.1f}", delta_color="normal" if margin > 0 else "inverse",
              help="Kelvin of headroom before the XLPE continuous limit. Expressed in "
                   "kelvin because it is a temperature difference, not a temperature.")
    c4.metric("Load", f"{load_factor*100:.0f} %",
              f"{load_factor*CABLE.rated_current:.0f} A")

    st.divider()
    st.subheader("Soil thermal resistivity along route")
    st.caption(
        "The mechanism behind the hotspot: elevated soil thermal "
        "resistivity raises T4 in the IEC 60287 chain, which raises "
        "conductor temperature at that location."
    )
    fig2 = go.Figure()
    fig2.add_trace(go.Scatter(x=distance, y=rho, mode="lines",
                               line=dict(color="#7F77DD", width=2),
                               name="Soil resistivity"))
    fig2.update_layout(xaxis_title="Distance from Station A (km)",
                        yaxis_title="Soil resistivity (K.m/W)",
                        height=240, margin=dict(t=20, b=10), showlegend=False)
    st.plotly_chart(fig2, use_container_width=True)

# -------------------------------------------------------------- SCADA
with tab_scada:
    st.subheader("Regional Control Center")
    st.caption("380 kV cable route temperature monitoring")

    st.markdown(
        f"""
        <div style="padding:16px;border-radius:8px;
                    background-color:{state_color}22;
                    border-left:5px solid {state_color};">
          <div style="font-size:13px;opacity:0.8;">ROUTE STATUS</div>
          <div style="font-size:30px;font-weight:500;color:{state_color};">
            {state_name}
          </div>
          <div style="font-size:14px;">
            Hotspot {peak_c:.1f} C at {hs_km:.1f} km from Station A
          </div>
        </div>
        """,
        unsafe_allow_html=True,
    )

    st.write("")
    left, right = st.columns([2, 1])

    with left:
        st.markdown("**Route overview**")
        fig3 = go.Figure()
        fig3.add_trace(go.Scatter(
            x=distance, y=np.zeros_like(distance), mode="lines",
            line=dict(color="#B4B2A9", width=8), showlegend=False,
            hoverinfo="skip",
        ))
        fig3.add_trace(go.Scatter(
            x=JOINT_LOCATIONS_KM, y=[0] * len(JOINT_LOCATIONS_KM),
            mode="markers", marker=dict(color="#5F5E5A", size=10),
            name="Joints",
        ))
        fig3.add_trace(go.Scatter(
            x=[0, ROUTE_LENGTH_KM], y=[0, 0], mode="markers+text",
            text=["Station A", "Station B"], textposition="bottom center",
            marker=dict(color="#185FA5", size=16, symbol="square"),
            showlegend=False,
        ))
        fig3.add_trace(go.Scatter(
            x=[hs_km], y=[0], mode="markers",
            marker=dict(color=state_color, size=20, symbol="diamond"),
            name="Hotspot",
        ))
        fig3.update_layout(
            height=190, margin=dict(t=10, b=10),
            xaxis_title="Distance from Station A (km)",
            yaxis=dict(visible=False, range=[-0.5, 0.5]),
            legend=dict(orientation="h", y=1.2),
        )
        st.plotly_chart(fig3, use_container_width=True)

    with right:
        st.markdown("**System status**")
        def indicator(label: str, ok: bool, on_text="Active", off_text="Clear"):
            dot = "#E24B4A" if ok else "#1D9E75"
            txt = on_text if ok else off_text
            st.markdown(
                f"<div style='display:flex;justify-content:space-between;"
                f"padding:5px 0;border-bottom:1px solid rgba(136,135,128,0.2);'>"
                f"<span>{label}</span>"
                f"<span style='color:{dot};font-weight:500;'>{txt}</span></div>",
                unsafe_allow_html=True,
            )

        indicator("Alarm beacon", status.beacon, "ON", "Off")
        indicator("Audible alarm", status.buzzer, "ON", "Off")
        indicator("XLPE limit", status.limit_exceeded, "Exceeded", "Within limit")
        indicator("Communications", not status.comms_ok, "Failed", "Healthy")
        st.caption(
            f"Alarm logic source: "
            f"{'OpenPLC runtime' if status.source == 'plc' else 'Python fallback'}"
        )

# ---------------------------------------------------------------- Log
with tab_log:
    st.subheader("Event log")
    st.caption("State transitions are logged with timestamps, as required "
               "for post-event analysis.")
    if st.session_state.event_log:
        df = pd.DataFrame(st.session_state.event_log)
        st.dataframe(df, use_container_width=True, hide_index=True)

        buf = io.StringIO()
        df.to_csv(buf, index=False)
        st.download_button("Download event log (CSV)", buf.getvalue(),
                            file_name="dts_event_log.csv", mime="text/csv")
        if st.button("Clear log"):
            st.session_state.event_log = []
            st.rerun()
    else:
        st.info("No events recorded yet.")
