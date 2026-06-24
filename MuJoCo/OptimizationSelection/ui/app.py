"""
app.py  —  Streamlit UI for Motor + GR Co-Design Optimizer
============================================================
Launch with:
    streamlit run OptimizationSelection/ui/app.py
"""

import os
import sys
import copy

import numpy as np
import pandas as pd
import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import yaml

# ── Path setup ───────────────────────────────────────────────────────────────
_UI_DIR   = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.dirname(_UI_DIR)
sys.path.insert(0, _ROOT_DIR)

from core.inverse_dynamics import compute_joint_profile
from core.requirements import extract_requirements
from core.evaluator import evaluate_all
from core.regen import compute_regen, simulate_cap_voltage
from core.scorer import score_candidates
from main import load_config, load_motors, load_gearboxes

# ── Page config ───────────────────────────────────────────────────────────────
st.set_page_config(
    page_title="Motor + GR Co-Design Tool",
    page_icon="⚡",
    layout="wide",
    initial_sidebar_state="expanded",
)

# ── Custom CSS ────────────────────────────────────────────────────────────────
st.markdown("""
<style>
  /* ─── Global font ─── */
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap');

  html, body, [class*="css"] {
    font-family: 'Inter', sans-serif;
  }

  /* ─── Dark gradient background ─── */
  .stApp {
    background: linear-gradient(135deg, #0d1117 0%, #161b22 50%, #0d1117 100%);
    color: #e6edf3;
  }

  /* ─── Sidebar ─── */
  [data-testid="stSidebar"] {
    background: linear-gradient(180deg, #161b22 0%, #1c2128 100%);
    border-right: 1px solid #30363d;
  }
  [data-testid="stSidebar"] .stMarkdown h2 {
    color: #58a6ff;
    font-weight: 700;
    letter-spacing: -0.3px;
  }

  /* ─── Metric cards ─── */
  [data-testid="stMetricValue"] {
    font-size: 1.6rem !important;
    font-weight: 700;
    color: #58a6ff;
  }
  [data-testid="stMetricLabel"] {
    font-size: 0.75rem;
    color: #8b949e;
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }

  /* ─── Section headers ─── */
  .section-header {
    background: linear-gradient(90deg, #1f6feb22, transparent);
    border-left: 3px solid #1f6feb;
    padding: 0.5rem 1rem;
    border-radius: 0 6px 6px 0;
    margin: 1rem 0 0.5rem 0;
    color: #58a6ff;
    font-weight: 600;
    font-size: 0.9rem;
    text-transform: uppercase;
    letter-spacing: 0.08em;
  }

  /* ─── Score badge ─── */
  .score-badge {
    display: inline-block;
    background: linear-gradient(135deg, #1f6feb, #0d47a1);
    color: white;
    padding: 0.2rem 0.6rem;
    border-radius: 12px;
    font-size: 0.85rem;
    font-weight: 600;
    font-family: 'JetBrains Mono', monospace;
  }

  /* ─── Pass/fail pills ─── */
  .pill-pass {
    background: #1a7f3c22;
    color: #3fb950;
    border: 1px solid #3fb950;
    border-radius: 999px;
    padding: 0.1rem 0.5rem;
    font-size: 0.75rem;
    font-weight: 600;
  }
  .pill-fail {
    background: #b6212122;
    color: #f85149;
    border: 1px solid #f85149;
    border-radius: 999px;
    padding: 0.1rem 0.5rem;
    font-size: 0.75rem;
    font-weight: 600;
  }

  /* ─── Tab styling ─── */
  [data-baseweb="tab"] {
    font-size: 0.85rem;
    font-weight: 500;
    color: #8b949e;
  }
  [aria-selected="true"][data-baseweb="tab"] {
    color: #58a6ff !important;
    border-bottom-color: #58a6ff !important;
  }

  /* ─── Expander ─── */
  [data-testid="stExpander"] {
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 8px;
  }

  /* ─── Divider ─── */
  hr { border-color: #30363d; }

  /* ─── Code / mono ─── */
  code {
    font-family: 'JetBrains Mono', monospace;
    font-size: 0.82rem;
    background: #0d1117;
    padding: 0.15rem 0.3rem;
    border-radius: 4px;
    color: #79c0ff;
  }

  /* ─── Plotly chart backgrounds ─── */
  .js-plotly-plot .plotly .modebar { background: transparent !important; }
</style>
""", unsafe_allow_html=True)

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG PATHS
# ─────────────────────────────────────────────────────────────────────────────
_CFG_PATH  = os.path.join(_ROOT_DIR, "config", "robot_params.yaml")
_MOT_PATH  = os.path.join(_ROOT_DIR, "config", "motors.yaml")
_GEAR_PATH = os.path.join(_ROOT_DIR, "config", "gearboxes.yaml")


# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────────────────────

PLOTLY_LAYOUT = dict(
    paper_bgcolor="rgba(13,17,23,0)",
    plot_bgcolor="rgba(13,17,23,0)",
    font=dict(family="Inter", color="#e6edf3", size=12),
    xaxis=dict(gridcolor="#21262d", showgrid=True, zeroline=False),
    yaxis=dict(gridcolor="#21262d", showgrid=True, zeroline=False),
    margin=dict(l=50, r=20, t=40, b=40),
    legend=dict(bgcolor="rgba(22,27,34,0.8)", bordercolor="#30363d", borderwidth=1),
)

COLOR_GRAD = px.colors.sequential.Blues_r


def _plotly_dark(fig: go.Figure, **extra) -> go.Figure:
    layout = {**PLOTLY_LAYOUT, **extra}
    fig.update_layout(**layout)
    return fig


@st.cache_data(show_spinner=False)
def cached_pipeline(cfg_json: str, motors_json: str, gearboxes_json: str):
    """
    Runs the full pipeline. Cached by stringified config so it only
    re-runs when inputs change.
    """
    import json
    cfg       = json.loads(cfg_json)
    motors    = json.loads(motors_json)
    gearboxes = json.loads(gearboxes_json)

    profile = compute_joint_profile(cfg)
    reqs    = extract_requirements(profile)
    all_cands = evaluate_all(motors, gearboxes, reqs, cfg)

    passed_elec = [c for c in all_cands if c.passed]
    passed_all  = []
    for c in passed_elec:
        rm = compute_regen(c, profile, reqs, cfg)
        c.regen_metrics = rm
        c.regen_ok = rm["regen_ok"]
        if rm["regen_ok"]:
            passed_all.append(c)
        else:
            c.passed = False
            c.fail_reasons.extend(rm["fail_reasons"])

    ranked = score_candidates(passed_all, cfg) if passed_all else []

    return profile, reqs, all_cands, passed_all, ranked


# ─────────────────────────────────────────────────────────────────────────────
#  SIDEBAR — INPUTS
# ─────────────────────────────────────────────────────────────────────────────

def build_sidebar() -> dict:
    """Renders the sidebar form and returns the assembled cfg dict."""

    with st.sidebar:
        st.markdown("## ⚙️ Configuration")

        # ── Joint selection ───────────────────────────────────────────────────
        st.markdown('<div class="section-header">Target Joint</div>', unsafe_allow_html=True)
        joint = st.selectbox(
            "Joint to optimize",
            ["knee_pitch", "hip_pitch", "ankle_pitch"],
            index=0,
            key="joint_select",
            help="Each joint has its own decentralized supercap"
        )

        # ── Gait profile ──────────────────────────────────────────────────────
        st.markdown('<div class="section-header">Gait Profile</div>', unsafe_allow_html=True)
        profile_source = st.selectbox(
            "Profile source",
            ["analytic", "online", "mujoco_csv"],
            index=0,
            key="profile_source",
        )
        gait_type = st.selectbox(
            "Gait type",
            ["step_in_place", "squat", "stair_climb", "stand"],
            index=0,
            key="gait_type",
        )
        col_a, col_b = st.columns(2)
        with col_a:
            step_freq = st.number_input("Step freq (Hz)", 0.3, 3.0, 1.0, 0.1, key="step_freq")
            step_h    = st.number_input("Step height (m)", 0.01, 0.30, 0.04, 0.01, key="step_h")
        with col_b:
            slope_deg  = st.number_input("Slope (°)", 0.0, 30.0, 0.0, 1.0, key="slope")
            stair_rise = st.number_input("Stair rise (m)", 0.05, 0.40, 0.18, 0.01, key="stair_rise")

        n_cycles = st.slider("Gait cycles for averaging", 1, 10, 3, key="n_cycles")

        mujoco_csv = ""
        if profile_source == "mujoco_csv":
            mujoco_csv = st.text_input("CSV path (from test_kbot_legs.py)", "",
                                       key="mujoco_csv")

        # ── Robot body ────────────────────────────────────────────────────────
        st.markdown('<div class="section-header">Robot Body</div>', unsafe_allow_html=True)
        col_r1, col_r2 = st.columns(2)
        with col_r1:
            robot_mass   = st.number_input("Robot mass (kg)", 10.0, 200.0, 32.0, 1.0, key="robot_mass")
        with col_r2:
            payload_mass = st.number_input("Payload (kg)",    0.0,  100.0, 10.0, 1.0, key="payload_mass")

        # ── Electrical ────────────────────────────────────────────────────────
        st.markdown('<div class="section-header">Electrical</div>', unsafe_allow_html=True)
        col_e1, col_e2 = st.columns(2)
        with col_e1:
            v_bus  = st.number_input("Bus voltage (V)", 24.0, 120.0, 96.0, 1.0, key="v_bus")
            i_max  = st.number_input("I_phase max (A)", 5.0, 200.0, 60.0, 5.0, key="i_max")
        with col_e2:
            v_margin     = st.slider("Voltage margin", 0.60, 0.95, 0.80, 0.01, key="v_margin")
            v_util_tgt   = st.slider("V_util target",  0.50, 0.90, 0.75, 0.01, key="v_util_tgt")
        p_cu_lim = st.number_input("Cu-loss limit (W)", 10.0, 300.0, 80.0, 5.0, key="p_cu_lim")

        # ── Supercap ──────────────────────────────────────────────────────────
        st.markdown('<div class="section-header">Supercapacitor (per joint)</div>', unsafe_allow_html=True)
        col_c1, col_c2 = st.columns(2)
        with col_c1:
            cap_C    = st.number_input("Capacitance (F)", 1.0, 500.0, 50.0, 5.0, key="cap_C")
            cap_esr  = st.number_input("ESR (Ω)", 0.001, 1.0, 0.01, 0.001, key="cap_esr",
                                       format="%.3f")
        with col_c2:
            cap_vmin = st.number_input("V_min (V)", 5.0, 80.0, 40.0, 1.0, key="cap_vmin")
            cap_vmax = st.number_input("V_max (V)", 10.0, 120.0, 60.0, 1.0, key="cap_vmax")
        cap_vnom    = st.number_input("V_nominal (V)", cap_vmin, cap_vmax,
                                       min(50.0, cap_vmax), 1.0, key="cap_vnom")
        regen_min   = st.number_input("Min regen power (W)", 0.5, 50.0, 5.0, 0.5, key="regen_min")

        # ── Scoring weights ───────────────────────────────────────────────────
        with st.expander("🎛  Scoring Weights", expanded=False):
            w_mass  = st.slider("Mass",               0.0, 1.0, 0.20, 0.05, key="w_mass")
            w_vutil = st.slider("Voltage utilisation", 0.0, 1.0, 0.20, 0.05, key="w_vutil")
            w_eff   = st.slider("Efficiency",          0.0, 1.0, 0.20, 0.05, key="w_eff")
            w_regen = st.slider("Regen match",         0.0, 1.0, 0.15, 0.05, key="w_regen")
            w_iner  = st.slider("Inertia",             0.0, 1.0, 0.15, 0.05, key="w_iner")
            w_gr    = st.slider("GR penalty",          0.0, 1.0, 0.10, 0.05, key="w_gr")

        top_n = st.slider("Show top N results", 3, 30, 10, key="top_n")

    # ── Assemble cfg dict ─────────────────────────────────────────────────────
    cfg = {
        "robot": {
            "total_mass_kg": robot_mass,
            "payload_mass_kg": payload_mass,
            "g": 9.81,
            "link_lengths_m": {"thigh": 0.3854, "shank": 0.2915, "foot": 0.0478},
            "link_masses_kg": {"thigh": 5.297, "shank": 1.685, "foot": 0.609},
            "com_offsets_m":  {"thigh": [-0.00318, -0.19441],
                               "shank": [ 0.02412, -0.10431],
                               "foot":  [ 0.01655, -0.03114]},
            "link_inertias_kgm2": {"thigh": 0.12715, "shank": 0.01466, "foot": 0.00189},
            "joint_limits_rad": {
                "hip_pitch":   [-1.047, 2.217],
                "knee_pitch":  [0.0, 2.705],
                "ankle_pitch": [-1.134, 0.262],
            },
        },
        "gait": {
            "target_joint": joint,
            "profile_source": profile_source,
            "mujoco_csv_path": mujoco_csv,
            "gait_type": gait_type,
            "step_freq_hz": step_freq,
            "step_height_m": step_h,
            "slope_deg": slope_deg,
            "stair_rise_m": stair_rise,
            "t_cycle_s": 1.0 / step_freq * 2,
            "dt_s": 0.002,
            "n_cycles": n_cycles,
        },
        "electrical": {
            "V_bus": v_bus,
            "V_bus_utilization_target": v_util_tgt,
            "I_phase_max_A": i_max,
            "voltage_margin": v_margin,
            "eta_drive": 0.97,
        },
        "supercap": {
            "capacitance_F": cap_C,
            "esr_ohm": cap_esr,
            "V_min": cap_vmin,
            "V_max": cap_vmax,
            "V_nominal": cap_vnom,
            "regen_min_power_W": regen_min,
            "regen_overvoltage_margin": 0.95,
        },
        "thermal": {"P_cu_limit_W": p_cu_lim},
        "scoring": {
            "weights": {
                "mass": w_mass,
                "voltage_utilization": w_vutil,
                "efficiency": w_eff,
                "regen_match": w_regen,
                "reflected_inertia": w_iner,
                "gr_penalty": w_gr,
            },
            "gr_penalty_low": 3.0,
            "gr_penalty_high": 30.0,
        },
        "output": {"top_n": top_n, "results_dir": "data/output"},
    }

    return cfg, top_n


# ─────────────────────────────────────────────────────────────────────────────
#  TAB 1 — JOINT REQUIREMENTS
# ─────────────────────────────────────────────────────────────────────────────

def tab_requirements(profile: dict, reqs: dict):
    st.markdown("### 📊 Joint Torque & Velocity Profile")

    # Metrics row
    c1, c2, c3, c4, c5 = st.columns(5)
    c1.metric("T_cont (RMS)", f"{reqs['T_cont_Nm']:.1f} N·m")
    c2.metric("T_peak",       f"{reqs['T_peak_Nm']:.1f} N·m")
    c3.metric("ω_max",        f"{reqs['omega_max_rads']:.2f} rad/s")
    c4.metric("E_regen",      f"{reqs['E_regen_J']:.2f} J")
    c5.metric("Regen %",      f"{reqs['regen_fraction']*100:.1f}%")

    st.markdown("---")

    t   = profile["time"]
    tau = profile["torque"]
    om  = profile["velocity"]
    pw  = profile["power"]

    # ── Torque + velocity subplot ─────────────────────────────────────────────
    fig = make_subplots(
        rows=3, cols=1,
        shared_xaxes=True,
        subplot_titles=["Joint Torque (N·m)", "Angular Velocity (rad/s)", "Mechanical Power (W)"],
        vertical_spacing=0.08,
    )

    fig.add_trace(go.Scatter(
        x=t, y=tau,
        name="Torque",
        line=dict(color="#58a6ff", width=1.5),
        fill="tozeroy", fillcolor="rgba(88,166,255,0.08)"
    ), row=1, col=1)
    fig.add_hline(y=reqs["T_cont_Nm"],  line_dash="dash", line_color="#3fb950",
                  annotation_text="T_cont", row=1, col=1)
    fig.add_hline(y=reqs["T_peak_Nm"],  line_dash="dot",  line_color="#f85149",
                  annotation_text="T_peak", row=1, col=1)
    fig.add_hline(y=-reqs["T_cont_Nm"], line_dash="dash", line_color="#3fb950", row=1, col=1)

    fig.add_trace(go.Scatter(
        x=t, y=om,
        name="ω",
        line=dict(color="#d2a8ff", width=1.5),
        fill="tozeroy", fillcolor="rgba(210,168,255,0.08)"
    ), row=2, col=1)

    # Power — colour negative regions differently
    pos_mask = pw >= 0
    neg_mask = pw < 0
    fig.add_trace(go.Scatter(
        x=t, y=np.where(pos_mask, pw, np.nan),
        name="Motoring power", line=dict(color="#58a6ff", width=1.5),
        fill="tozeroy", fillcolor="rgba(88,166,255,0.08)"
    ), row=3, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=np.where(neg_mask, pw, np.nan),
        name="Regen power", line=dict(color="#3fb950", width=1.5),
        fill="tozeroy", fillcolor="rgba(63,185,80,0.15)"
    ), row=3, col=1)

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    _plotly_dark(fig, height=520, title_text="")
    st.plotly_chart(fig, use_container_width=True)

    # ── Torque–speed plane ────────────────────────────────────────────────────
    st.markdown("#### Operating Envelope (τ–ω plane)")
    fig2 = go.Figure()
    fig2.add_trace(go.Scatter(
        x=np.abs(om), y=np.abs(tau),
        mode="markers",
        marker=dict(
            color=pw, colorscale="RdYlGn", cmin=-200, cmax=200,
            size=3, opacity=0.6,
            colorbar=dict(title="Power (W)", thickness=12, len=0.6)
        ),
        name="Operating points",
        hovertemplate="ω=%{x:.3f} rad/s<br>τ=%{y:.2f} N·m<extra></extra>"
    ))
    fig2.add_vline(x=reqs["omega_max_rads"], line_dash="dash", line_color="#f85149",
                   annotation_text="ω_max")
    fig2.add_hline(y=reqs["T_peak_Nm"], line_dash="dot", line_color="#f85149",
                   annotation_text="T_peak")
    fig2.update_xaxes(title_text="|ω| (rad/s)")
    fig2.update_yaxes(title_text="|τ| (N·m)")
    _plotly_dark(fig2, height=320, title_text="Torque–Speed Operating Cloud")
    st.plotly_chart(fig2, use_container_width=True)


# ─────────────────────────────────────────────────────────────────────────────
#  TAB 2 — CANDIDATE SWEEP
# ─────────────────────────────────────────────────────────────────────────────

def tab_sweep(all_candidates: list, ranked: list, reqs: dict):
    st.markdown("### 🔍 Candidate Sweep Overview")

    total = len(all_candidates)
    passed_elec = sum(1 for c in all_candidates if c.passed or c.regen_ok)
    passed_all  = len(ranked)
    failed      = total - passed_all

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Total candidates", f"{total}")
    col2.metric("Passed all constraints", f"{passed_all}",
                delta=f"{passed_all/total*100:.0f}%")
    col3.metric("Failed", f"{failed}")
    col4.metric("Top score", f"{ranked[0]['score_total']:.4f}" if ranked else "—")

    if not ranked:
        st.warning("No candidates passed all constraints. Try relaxing parameters in the sidebar.")
        return

    # Build dataframe from ranked
    df = pd.DataFrame(ranked)

    # ── Scatter: Score vs Mass, coloured by GR ───────────────────────────────
    st.markdown("#### Score vs Total Mass — coloured by Gear Ratio")
    fig = px.scatter(
        df, x="total_mass_kg", y="score_total",
        color="GR", hover_data=["motor_id", "gear_id", "GR",
                                  "V_utilisation", "P_cu_W", "J_ref"],
        color_continuous_scale="Blues",
        labels={"total_mass_kg": "Total Mass (kg)",
                "score_total": "Score",
                "GR": "Gear Ratio"},
        size=[8]*len(df),
    )
    fig.update_traces(marker=dict(size=9, line=dict(width=0.5, color="#30363d")))
    _plotly_dark(fig, height=380, coloraxis_colorbar=dict(thickness=14, len=0.6))
    st.plotly_chart(fig, use_container_width=True)

    # ── Parallel coordinates ──────────────────────────────────────────────────
    st.markdown("#### Multi-Dimensional View")
    df_pc = df[["motor_id", "GR", "total_mass_kg",
                 "V_utilisation", "P_cu_W", "J_ref",
                 "regen_power_mean_W", "score_total"]].copy()
    df_pc["V_util_%"] = df_pc["V_utilisation"] * 100
    df_pc["J_ref_e3"] = df_pc["J_ref"] * 1000

    dims = [
        dict(label="GR",       values=df_pc["GR"]),
        dict(label="Mass (kg)",values=df_pc["total_mass_kg"]),
        dict(label="V_util %", values=df_pc["V_util_%"]),
        dict(label="P_cu (W)", values=df_pc["P_cu_W"]),
        dict(label="J_ref×1e3",values=df_pc["J_ref_e3"]),
        dict(label="Regen (W)",values=df_pc["regen_power_mean_W"]),
        dict(label="Score",    values=df_pc["score_total"]),
    ]
    fig_pc = go.Figure(go.Parcoords(
        line=dict(
            color=df_pc["score_total"],
            colorscale="Blues",
            showscale=True,
            cmin=df_pc["score_total"].min(),
            cmax=df_pc["score_total"].max(),
            colorbar=dict(title="Score", thickness=14, len=0.6)
        ),
        dimensions=dims,
    ))
    _plotly_dark(fig_pc, height=380)
    st.plotly_chart(fig_pc, use_container_width=True)

    # ── Failure breakdown ─────────────────────────────────────────────────────
    st.markdown("#### Failure Analysis")
    fail_counts = {}
    for c in all_candidates:
        for reason in c.fail_reasons:
            # Extract category from reason string
            cat = reason.split(":")[0].strip()
            fail_counts[cat] = fail_counts.get(cat, 0) + 1

    if fail_counts:
        fig_fail = go.Figure(go.Bar(
            x=list(fail_counts.values()),
            y=list(fail_counts.keys()),
            orientation="h",
            marker=dict(color="#f85149", opacity=0.75),
        ))
        fig_fail.update_xaxes(title_text="Count of failed candidates")
        _plotly_dark(fig_fail, height=260, title_text="Constraint Failure Breakdown")
        st.plotly_chart(fig_fail, use_container_width=True)


# ─────────────────────────────────────────────────────────────────────────────
#  TAB 3 — TOP RESULTS TABLE
# ─────────────────────────────────────────────────────────────────────────────

def tab_results(ranked: list, top_n: int, motors_lib: list, gearboxes_lib: list, cfg: dict):
    st.markdown("### 🏆 Top Candidates")

    if not ranked:
        st.info("No candidates to show.")
        return

    motor_lookup = {m["id"]: m for m in motors_lib}
    gear_lookup  = {g["id"]: g for g in gearboxes_lib}

    for i, r in enumerate(ranked[:top_n]):
        score_pct = f"{r['score_total']*100:.1f}"
        motor_name = motor_lookup.get(r["motor_id"], {}).get("name", r["motor_id"])
        gear_name  = gear_lookup.get(r["gear_id"],   {}).get("name", r["gear_id"])

        with st.expander(
            f"**#{i+1}**  {motor_name}  +  {gear_name}  │  "
            f"GR = {r['GR']:.1f}  │  Score = {score_pct}%",
            expanded=(i == 0)
        ):
            col_a, col_b, col_c = st.columns(3)

            with col_a:
                st.markdown("**⚡ Electrical**")
                st.write(f"- Bus utilisation: `{r['V_utilisation']*100:.1f}%`")
                st.write(f"- V_emf: `{r['V_emf']:.1f} V`")
                st.write(f"- V_total: `{r['V_total']:.1f} V`")
                st.write(f"- I_rms: `{r['I_rms']:.2f} A`")
                st.write(f"- I_peak: `{r['I_peak']:.2f} A`")
                st.write(f"- Cu loss: `{r['P_cu_W']:.1f} W`")

            with col_b:
                st.markdown("**🔧 Mechanical**")
                st.write(f"- Motor ω_max: `{r['omega_motor_max']:.1f} rad/s`")
                st.write(f"- T_motor_cont: `{r['T_motor_cont']:.2f} N·m`")
                st.write(f"- T_motor_peak: `{r['T_motor_peak']:.2f} N·m`")
                st.write(f"- J_ref: `{r['J_ref']*1000:.3f} ×10⁻³ kg·m²`")
                st.write(f"- Total mass: `{r['total_mass_kg']:.3f} kg`")
                st.write(f"- Bandwidth score: `{r['bandwidth_score']:.3f}`")

            with col_c:
                st.markdown("**🔋 Regen / Supercap**")
                st.write(f"- Regen power (mean): `{r['regen_power_mean_W']:.1f} W`")
                st.write(f"- Regen energy: `{r['regen_energy_J']:.2f} J`")
                st.write(f"- V_cap peak: `{r['V_cap_peak_V']:.1f} V`")
                st.write(f"- Cap match score: `{r['cap_power_match_score']:.3f}`")

            # Score radar
            st.markdown("**📊 Score Breakdown**")
            categories = ["Mass", "V_util", "Efficiency", "Regen", "Inertia", "GR"]
            vals = [r["score_mass"], r["score_v_util"], r["score_efficiency"],
                    r["score_regen"], r["score_inertia"], r["score_gr"]]
            vals_closed = vals + [vals[0]]
            cats_closed = categories + [categories[0]]

            fig_r = go.Figure(go.Scatterpolar(
                r=vals_closed, theta=cats_closed,
                fill="toself",
                fillcolor="rgba(88,166,255,0.15)",
                line=dict(color="#58a6ff", width=2),
                name=f"#{i+1}",
            ))
            fig_r.update_layout(
                polar=dict(
                    bgcolor="rgba(13,17,23,0)",
                    radialaxis=dict(visible=True, range=[0, 1],
                                   gridcolor="#30363d", linecolor="#30363d",
                                   tickfont=dict(color="#8b949e", size=9)),
                    angularaxis=dict(gridcolor="#30363d", linecolor="#30363d",
                                     tickfont=dict(color="#e6edf3", size=11)),
                ),
                paper_bgcolor="rgba(13,17,23,0)",
                plot_bgcolor="rgba(13,17,23,0)",
                font=dict(color="#e6edf3", family="Inter"),
                showlegend=False,
                height=280,
                margin=dict(l=40, r=40, t=20, b=20),
            )
            st.plotly_chart(fig_r, use_container_width=True)

    # ── Summary table ─────────────────────────────────────────────────────────
    st.markdown("#### 📋 Summary Table")
    cols_show = ["motor_id", "gear_id", "GR", "score_total",
                 "total_mass_kg", "V_utilisation", "P_cu_W", "J_ref",
                 "regen_power_mean_W", "V_cap_peak_V", "eta_total"]
    df_show = pd.DataFrame(ranked[:top_n])[cols_show].copy()
    df_show["V_util_%"]  = (df_show["V_utilisation"] * 100).round(1)
    df_show["J_ref_e3"]  = (df_show["J_ref"] * 1000).round(4)
    df_show["score_%"]   = (df_show["score_total"] * 100).round(2)
    df_show["η_total"]   = (df_show["eta_total"] * 100).round(1)
    df_show = df_show.rename(columns={
        "motor_id": "Motor", "gear_id": "Gear",
        "total_mass_kg": "Mass (kg)", "P_cu_W": "P_cu (W)",
        "regen_power_mean_W": "P_regen (W)", "V_cap_peak_V": "V_cap (V)"
    })
    st.dataframe(df_show.drop(columns=["J_ref", "V_utilisation", "score_total", "eta_total"]),
                 use_container_width=True, height=350)

    # Download button
    csv_bytes = df_show.to_csv(index=False).encode()
    st.download_button(
        "⬇  Download CSV",
        data=csv_bytes,
        file_name=f"results_{cfg['gait']['target_joint']}.csv",
        mime="text/csv",
    )


# ─────────────────────────────────────────────────────────────────────────────
#  TAB 4 — REGEN ANALYSIS
# ─────────────────────────────────────────────────────────────────────────────

def tab_regen(ranked: list, profile: dict, cfg: dict, top_n: int):
    st.markdown("### 🔋 Supercap Regen Analysis")

    if not ranked:
        st.info("No passing candidates. Cannot show regen analysis.")
        return

    n_show = min(top_n, 5, len(ranked))
    candidates_show = [f"#{i+1}  {r['motor_id']} + {r['gear_id']} (GR={r['GR']:.1f})"
                       for i, r in enumerate(ranked[:n_show])]

    sel = st.selectbox("Select candidate", candidates_show, key="regen_sel")
    idx = int(sel.split("#")[1].split(" ")[0]) - 1
    r   = ranked[idx]

    eta_regen = st.slider("Regen efficiency (η)", 0.30, 0.95, 0.70, 0.05,
                           key="eta_regen_slider")

    # Simulate cap voltage
    sim = simulate_cap_voltage(profile, cfg, eta_regen=eta_regen)

    col_r1, col_r2, col_r3, col_r4 = st.columns(4)
    col_r1.metric("Mean regen power", f"{r['regen_power_mean_W']:.1f} W")
    col_r2.metric("Peak regen power", f"{r['regen_power_peak_W']:.1f} W")
    col_r3.metric("Regen energy",     f"{r['regen_energy_J']:.2f} J")
    col_r4.metric("Cap V peak",        f"{r['V_cap_peak_V']:.1f} V")

    # ── Cap voltage trajectory ────────────────────────────────────────────────
    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True,
        subplot_titles=["Cap Voltage (V)", "Regen Current (A)", "Regen Power (W)"],
        vertical_spacing=0.08,
    )

    vmin = cfg["supercap"]["V_min"]
    vmax = cfg["supercap"]["V_max"]
    vnom = cfg["supercap"]["V_nominal"]

    fig.add_trace(go.Scatter(
        x=sim["time"], y=sim["V_cap"], name="V_cap",
        line=dict(color="#58a6ff", width=2)
    ), row=1, col=1)
    fig.add_hline(y=vmax, line_dash="dot", line_color="#f85149",
                  annotation_text="V_max", row=1, col=1)
    fig.add_hline(y=vmin, line_dash="dot", line_color="#f85149",
                  annotation_text="V_min", row=1, col=1)
    fig.add_hline(y=vnom, line_dash="dash", line_color="#d2a8ff",
                  annotation_text="V_nom", row=1, col=1)

    i_cap = sim["I_cap"]
    fig.add_trace(go.Scatter(
        x=sim["time"],
        y=np.where(i_cap > 0, i_cap, np.nan),
        name="Regen I", line=dict(color="#3fb950", width=1.5),
        fill="tozeroy", fillcolor="rgba(63,185,80,0.12)"
    ), row=2, col=1)
    fig.add_trace(go.Scatter(
        x=sim["time"],
        y=np.where(i_cap < 0, i_cap, np.nan),
        name="Discharge I", line=dict(color="#f85149", width=1.5),
        fill="tozeroy", fillcolor="rgba(248,81,73,0.10)"
    ), row=2, col=1)

    fig.add_trace(go.Scatter(
        x=sim["time"], y=sim["P_regen"], name="P_regen",
        line=dict(color="#ffa657", width=1.5),
        fill="tozeroy", fillcolor="rgba(255,166,87,0.12)"
    ), row=3, col=1)

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    _plotly_dark(fig, height=520, title_text="Supercap Voltage & Regen Trajectory")
    st.plotly_chart(fig, use_container_width=True)

    # ── Regen energy pie ──────────────────────────────────────────────────────
    E_total    = float(np.trapz(np.abs(profile["power"]), profile["time"]))
    E_regen    = r["regen_energy_J"]
    E_motoring = E_total - E_regen

    st.markdown("#### Energy Split per Gait Period")
    fig_pie = go.Figure(go.Pie(
        labels=["Motoring energy", "Recoverable regen energy"],
        values=[E_motoring, E_regen],
        hole=0.55,
        marker=dict(colors=["#1f6feb", "#3fb950"],
                    line=dict(color="#0d1117", width=2)),
        textfont=dict(color="#e6edf3", size=13),
    ))
    _plotly_dark(fig_pie, height=300,
                 title_text=f"Total cycle energy: {E_total:.1f} J")
    st.plotly_chart(fig_pie, use_container_width=True)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN LAYOUT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # Header
    st.markdown(
        """
        <div style='text-align:center; padding: 1.5rem 0 0.5rem 0;'>
          <h1 style='font-size:2rem; font-weight:800; letter-spacing:-0.5px;
                     background: linear-gradient(90deg,#58a6ff,#3fb950);
                     -webkit-background-clip:text; -webkit-text-fill-color:transparent;'>
            ⚡ Motor + GR Co-Design Tool
          </h1>
          <p style='color:#8b949e; font-size:0.92rem; margin-top:0.2rem;'>
            96V DC bus · Decentralized supercap regen · Per-joint optimization
          </p>
        </div>
        """,
        unsafe_allow_html=True
    )

    # Build sidebar and get config
    cfg, top_n = build_sidebar()

    # Load component libraries
    motors_lib    = load_motors(_MOT_PATH)
    gearboxes_lib = load_gearboxes(_GEAR_PATH)

    # Run pipeline (cached)
    import json
    cfg_json   = json.dumps(cfg,   sort_keys=True)
    motor_json = json.dumps(motors_lib,    sort_keys=True)
    gear_json  = json.dumps(gearboxes_lib, sort_keys=True)

    with st.spinner("Running optimization pipeline…"):
        try:
            profile, reqs, all_cands, passed_all, ranked = cached_pipeline(
                cfg_json, motor_json, gear_json
            )
        except Exception as e:
            st.error(f"Pipeline error: {e}")
            import traceback
            st.code(traceback.format_exc(), language="python")
            return

    # ── Status bar ───────────────────────────────────────────────────────────
    total     = len(all_cands)
    n_pass    = len(ranked)
    frac_pass = n_pass / total * 100 if total > 0 else 0

    status_col = "#3fb950" if n_pass > 0 else "#f85149"
    st.markdown(
        f"""
        <div style='display:flex; gap:1.5rem; padding:0.7rem 1rem;
                    background:#161b22; border-radius:8px; border:1px solid #30363d;
                    margin-bottom:1rem; align-items:center;'>
          <span style='color:{status_col}; font-weight:700; font-size:1.2rem;'>
            {'✅' if n_pass > 0 else '❌'}
          </span>
          <span style='color:#e6edf3;'>
            <b>{n_pass}</b> / {total} candidates passed all constraints
            ({frac_pass:.0f}%)  ·  Joint: <code>{cfg['gait']['target_joint']}</code>
            ·  Profile: <code>{cfg['gait']['profile_source']}</code>
          </span>
        </div>
        """,
        unsafe_allow_html=True
    )

    # ── Tabs ─────────────────────────────────────────────────────────────────
    tab1, tab2, tab3, tab4 = st.tabs([
        "📊 Joint Requirements",
        "🔍 Candidate Sweep",
        "🏆 Top Results",
        "🔋 Regen Analysis",
    ])

    with tab1:
        tab_requirements(profile, reqs)
    with tab2:
        tab_sweep(all_cands, ranked, reqs)
    with tab3:
        tab_results(ranked, top_n, motors_lib, gearboxes_lib, cfg)
    with tab4:
        tab_regen(ranked, profile, cfg, top_n)


if __name__ == "__main__":
    main()
