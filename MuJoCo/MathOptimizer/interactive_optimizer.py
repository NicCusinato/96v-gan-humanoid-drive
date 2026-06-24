import streamlit as st
import numpy as np
import plotly.graph_objects as go
from motor_math_optimizer import voltage_limit_R, thermal_limit_R, optimize_motor, generate_feasible_space

st.set_page_config(page_title="96V GaN Motor Synthesizer", layout="wide", initial_sidebar_state="expanded")

st.title("⚡ 96V GaN Motor Parameter Synthesizer")
st.markdown("""
This tool analytically synthesizes the **Ideal Motor Parameters** ($K_t$, Phase Resistance $R$, and $K_m$) 
for a high-performance humanoid joint driven by a 96V GaN inverter. 

Instead of selecting from a discrete catalog, this tool models the continuous mathematical boundaries:
1. **Thermal Limit:** $I_{rms}^2 R \le P_{thermal}$
2. **Voltage Ceiling:** $(T_{peak}/K_t) R + K_t \omega_{peak} \le V_{bus}$

The optimal motor exactly rides the intersection of these two constraints, maximizing $R$ (minimizing copper mass) while delivering the required dynamic performance.
""")

# --- Sidebar Inputs ---
st.sidebar.header("Dynamic Requirements")
T_peak = st.sidebar.slider("Peak Torque $T_{peak}$ (Nm)", 10.0, 300.0, 84.0, 1.0)
T_rms = st.sidebar.slider("RMS Torque $T_{rms}$ (Nm)", 5.0, 100.0, 20.0, 1.0)
omega_peak = st.sidebar.slider("Peak Velocity $\omega_{peak}$ (rad/s)", 1.0, 50.0, 20.0, 0.5)

st.sidebar.header("Electrical & Thermal Constraints")
V_bus = st.sidebar.slider("DC Bus Voltage $V_{bus}$ (V)", 24.0, 120.0, 96.0, 1.0)
P_thermal = st.sidebar.slider("Max Thermal Dissipation (W)", 10.0, 500.0, 100.0, 5.0)

# --- Optimization ---
st.subheader("1. Optimal Motor Synthesis")

opt_res = optimize_motor(T_peak, T_rms, omega_peak, V_bus, P_thermal)
Kt_opt = opt_res['Kt']
R_opt = opt_res['R']
Km_opt = opt_res['Km']
Kv_opt = 9.55 / Kt_opt if Kt_opt > 0 else 0

if opt_res['success'] and R_opt > 0:
    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Optimal $K_t$ (Nm/A)", f"{Kt_opt:.3f}")
    col2.metric("Optimal Phase $R$ (Ω)", f"{R_opt:.3f}")
    col3.metric("Motor Constant $K_m$", f"{Km_opt:.3f}")
    col4.metric("Equivalent $K_v$ (RPM/V)", f"{Kv_opt:.1f}")
    
    st.success(f"**Synthesis Successful:** This motor will exactly consume {opt_res['V_utilization']*100:.1f}% of the {V_bus}V bus at {omega_peak} rad/s and dissipate exactly {opt_res['P_utilization']*100:.1f}% of the {P_thermal}W thermal limit at {T_rms} Nm RMS.")
else:
    st.error("No feasible motor found for these constraints! Try relaxing the dynamic requirements or increasing bus voltage / thermal limit.")

# --- Feasible Space Plot ---
st.subheader("2. Feasible Design Space ($K_t$ vs Phase $R$)")

# Generate bounds for plotting around the optimal point
Kt_max_plot = min(2.0, (V_bus / omega_peak) * 1.2)
Kt_range = (0.01, Kt_max_plot)
R_range = (0.001, max(0.1, R_opt * 2.5))

Kt_vals, R_vals, V_req, P_diss, feasible_both = generate_feasible_space(T_peak, T_rms, omega_peak, V_bus, P_thermal, Kt_range, R_range, resolution=200)

fig = go.Figure()

# Thermal Limit Curve (R = P_thermal * (Kt / T_rms)^2)
R_thermal_curve = thermal_limit_R(Kt_vals, T_rms, P_thermal)
fig.add_trace(go.Scatter(x=Kt_vals, y=R_thermal_curve, mode='lines', name='Thermal Limit', line=dict(color='red', dash='dash')))

# Voltage Limit Curve (R = (V_bus - Kt * omega_peak) * (Kt / T_peak))
R_voltage_curve = voltage_limit_R(Kt_vals, T_peak, omega_peak, V_bus)
# Mask negative values for clean plotting
R_voltage_curve[R_voltage_curve < 0] = None 
fig.add_trace(go.Scatter(x=Kt_vals, y=R_voltage_curve, mode='lines', name='Voltage Limit', line=dict(color='blue', dash='dash')))

# Add Optimal Point
if opt_res['success'] and R_opt > 0:
    fig.add_trace(go.Scatter(x=[Kt_opt], y=[R_opt], mode='markers', name='Optimal Motor', 
                             marker=dict(color='gold', size=15, symbol='star', line=dict(width=2, color='black'))))

# Shade Feasible Region
# We fill the area between the x-axis and the minimum of the two curves
R_feasible_upper = np.minimum(R_thermal_curve, np.nan_to_num(R_voltage_curve, nan=0.0))
fig.add_trace(go.Scatter(
    x=np.concatenate([Kt_vals, Kt_vals[::-1]]),
    y=np.concatenate([R_feasible_upper, np.zeros_like(R_feasible_upper)]),
    fill='toself',
    fillcolor='rgba(0, 255, 0, 0.2)',
    line=dict(color='rgba(255,255,255,0)'),
    name='Feasible Design Space',
    showlegend=True
))

fig.update_layout(
    xaxis_title="Torque Constant $K_t$ (Nm/A)",
    yaxis_title="Phase Resistance $R$ (Ω)",
    yaxis_range=[0, R_range[1]],
    xaxis_range=[0, Kt_range[1]],
    template="plotly_dark",
    legend=dict(yanchor="top", y=0.99, xanchor="right", x=0.99)
)

st.plotly_chart(fig, use_container_width=True)

st.markdown("""
### How to read this plot:
- **Red Dashed Line:** Represents the absolute thermal limit. Any motor with a resistance *above* this line will overheat (dissipate > $P_{thermal}$) when outputting the RMS torque.
- **Blue Dashed Line:** Represents the 96V bus ceiling. Any motor with a resistance *above* this line will run out of voltage (saturation) before reaching $T_{peak}$ at $\omega_{peak}$.
- **Green Shaded Area:** The feasible design space where both constraints are met.
- **Gold Star:** The mathematically optimal motor. It maximizes phase resistance $R$ (which correlates with minimizing copper mass and overall weight) while sitting perfectly on the edge of your thermal and voltage limits.

---

### Why 96V + GaN is powerful here:
If you drop the $V_{bus}$ slider to 48V, watch the Blue Voltage Curve collapse. The feasible space shrinks dramatically, forcing you to use motors with extremely low $K_t$ (which drastically increases current $I$, requiring massive phase wires) or extremely low Resistance (requiring a massive, heavy motor). **The 96V ceiling mathematically unlocks lightweight, high-$K_t$ motor geometries.**
""")
