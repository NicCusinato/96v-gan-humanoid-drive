import numpy as np
from scipy.optimize import minimize

def voltage_limit_R(Kt, T_peak, omega_peak, V_bus):
    """
    Calculates the maximum phase resistance R allowed by the voltage ceiling.
    V_req = (T_peak / Kt) * R + Kt * omega_peak <= V_bus
    => R <= (V_bus - Kt * omega_peak) * (Kt / T_peak)
    """
    # If back-EMF alone exceeds V_bus, no positive R is allowed
    back_emf = Kt * omega_peak
    if np.any(back_emf >= V_bus):
        # Handle scalar or array inputs
        if np.isscalar(Kt):
            return 0.0
        else:
            R = (V_bus - back_emf) * (Kt / T_peak)
            R[back_emf >= V_bus] = 0.0
            return np.maximum(0.0, R)
    
    return np.maximum(0.0, (V_bus - back_emf) * (Kt / T_peak))

def thermal_limit_R(Kt, T_rms, P_thermal_max):
    """
    Calculates the maximum phase resistance R allowed by the thermal limit.
    P_diss = (T_rms / Kt)^2 * R <= P_thermal_max
    => R <= P_thermal_max * (Kt / T_rms)^2
    """
    return P_thermal_max * (Kt / T_rms)**2

def optimize_motor(T_peak, T_rms, omega_peak, V_bus, P_thermal_max, weight_Km=1.0, weight_R=1.0):
    """
    Finds the optimal [Kt, R] that maximizes a combination of R (which minimizes mass) 
    and Km, while satisfying voltage and thermal constraints.
    
    We want to maximize R (thinner wire, less copper mass) while keeping Kt reasonable.
    Alternatively, we can just find the point where Voltage and Thermal limits intersect, 
    as this gives the absolute highest R (lowest mass) that still satisfies both at exactly 100%.
    """
    
    # The absolute best motor (lowest copper mass) is precisely where the 
    # voltage limit curve and thermal limit curve intersect.
    # At this intersection, the motor uses exactly 100% of V_bus at peak load,
    # and exactly 100% of P_thermal at RMS load.
    
    # We solve for Kt where voltage_limit_R(Kt) == thermal_limit_R(Kt)
    # P_thermal * (Kt / T_rms)^2 = (V_bus - Kt * omega_peak) * (Kt / T_peak)
    
    def objective(Kt):
        R_v = voltage_limit_R(Kt[0], T_peak, omega_peak, V_bus)
        R_t = thermal_limit_R(Kt[0], T_rms, P_thermal_max)
        # We want R_v and R_t to be as close as possible, while maximizing them
        # Minimizing the squared difference finds the intersection
        return (R_v - R_t)**2
    
    # Initial guess for Kt:
    # A reasonable guess is where back-EMF is half of V_bus
    Kt_guess = (V_bus / 2.0) / (omega_peak + 1e-9)
    
    # Bounds for Kt: back-EMF must be less than V_bus
    Kt_max = V_bus / (omega_peak + 1e-9)
    bounds = [(1e-4, Kt_max)]
    
    res = minimize(objective, [Kt_guess], bounds=bounds, method='Nelder-Mead')
    
    Kt_opt = res.x[0]
    R_opt = thermal_limit_R(Kt_opt, T_rms, P_thermal_max)
    Km_opt = Kt_opt / np.sqrt(R_opt) if R_opt > 0 else 0
    
    return {
        'Kt': Kt_opt,
        'R': R_opt,
        'Km': Km_opt,
        'success': res.success,
        'V_utilization': ((T_peak/Kt_opt)*R_opt + Kt_opt*omega_peak) / V_bus,
        'P_utilization': ((T_rms/Kt_opt)**2 * R_opt) / P_thermal_max
    }

def generate_feasible_space(T_peak, T_rms, omega_peak, V_bus, P_thermal_max, Kt_range=(0.01, 2.0), R_range=(0.01, 2.0), resolution=100):
    """
    Generates a 2D grid for contour plotting the feasible design space.
    """
    Kt_vals = np.linspace(Kt_range[0], Kt_range[1], resolution)
    R_vals = np.linspace(R_range[0], R_range[1], resolution)
    
    Kt_grid, R_grid = np.meshgrid(Kt_vals, R_vals)
    
    # Calculate constraints
    V_req = (T_peak / Kt_grid) * R_grid + Kt_grid * omega_peak
    P_diss = (T_rms / Kt_grid)**2 * R_grid
    
    # 1 if feasible, 0 if violated
    feasible_V = (V_req <= V_bus).astype(int)
    feasible_P = (P_diss <= P_thermal_max).astype(int)
    feasible_both = feasible_V & feasible_P
    
    return Kt_vals, R_vals, V_req, P_diss, feasible_both
