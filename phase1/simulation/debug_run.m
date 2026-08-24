% Debug script — inspect PMSM simlog fields to find iron vs copper split
clc; clear; close all;

modelName = 'VirtualSpeedDyno';
if ~bdIsLoaded(modelName), load_system(modelName); end

mc.lambda = 0.0075; mc.p = 14; mc.L_d = 95e-6; mc.L_q = 115e-6;
mc.R = 0.185; mc.V_bat = 48.0;
T_cmd = 2.5; w_cmd = 15.0; fsw_val = 40000;
gan_fets = {'Q1','Q2','Q3','Q4','Q5','Q6'};
R_cable_total = 0.016;
ss_thresh = 0.040;

fprintf('--- Running rapid-accelerator and checking corrected efficiency ---\n');
simIn = buildSimIn_debug(modelName, mc, T_cmd, w_cmd, fsw_val);
outRapid = sim(simIn);

if ~isempty(outRapid.ErrorMessage)
    fprintf('ERROR: %s\n', outRapid.ErrorMessage); return;
end

sl = outRapid.simlog;
time_vec = sl.Converter_Three_Phase.Q1.accumulatedSwitchingLosses.series.time;
ss_start = find(time_vec >= ss_thresh, 1);
ss_dt    = time_vec(end) - time_vec(ss_start);

fprintf('Time range: %.4f → %.4f s  |  SS window: %.4f → %.4f s (%d samples)\n', ...
    time_vec(1), time_vec(end), time_vec(ss_start), time_vec(end), numel(time_vec)-ss_start+1);

% --- Inverter losses (with flat-signal fallback) ---
inv_sw_loss = 0; inv_cond_loss = 0;
for q = 1:numel(gan_fets)
    sw_E = sl.Converter_Three_Phase.(gan_fets{q}).accumulatedSwitchingLosses.series.values('J');
    t_sw = sl.Converter_Three_Phase.(gan_fets{q}).accumulatedSwitchingLosses.series.time;
    delta = sw_E(end) - sw_E(ss_start);
    if delta > 1e-9 * sw_E(end)
        inv_sw_loss = inv_sw_loss + delta / ss_dt;
    else
        inv_sw_loss = inv_sw_loss + sw_E(end) / t_sw(end);
        if strcmp(gan_fets{q}, 'Q1')
            fprintf('Q1: flat SS window, using total avg: %.4f mW (total E=%.6f J, t=%.4f s)\n', ...
                (sw_E(end)/t_sw(end))*1e3, sw_E(end), t_sw(end));
        end
    end
    cond_P = sl.Converter_Three_Phase.(gan_fets{q}).power_dissipated.series.values('W');
    inv_cond_loss = inv_cond_loss + mean(cond_P(ss_start:end));
end
inv_loss = inv_sw_loss + inv_cond_loss;

% --- Motor losses ---
copper_P   = sl.PMSM.power_dissipated.series.values('W');
copper_loss = mean(copper_P(ss_start:end));

% --- Cable losses ---
i_dc = sl.Resistor.i.series.values('A');
cable_loss = mean((i_dc(ss_start:end).^2) .* R_cable_total);

% --- Mechanical power (negated — Simscape reaction torque convention) ---
T_mech = sl.PMSM.torque.series.values('N*m');
w_mech = sl.PMSM.angular_velocity.series.values('rad/s');
n_ss = numel(T_mech) - ss_start + 1;
late_start = ss_start + floor(n_ss * 0.5);
late_start = max(late_start, ss_start + 1);
P_mech = -mean(T_mech(late_start:end) .* w_mech(late_start:end));

% --- Efficiency ---
P_loss_total = inv_loss + copper_loss + 0 + cable_loss;  % iron_loss = 0
P_elec = P_mech + P_loss_total;

fprintf('\n--- Power Budget ---\n');
fprintf('  P_mech        = %7.3f W  (commanded: %.1f W)\n', P_mech, T_cmd*w_cmd);
fprintf('  Inv SW loss   = %7.3f W\n', inv_sw_loss);
fprintf('  Inv Cond loss = %7.3f W\n', inv_cond_loss);
fprintf('  Copper loss   = %7.3f W\n', copper_loss);
fprintf('  Cable loss    = %7.3f W\n', cable_loss);
fprintf('  P_elec (in)   = %7.3f W\n', P_elec);
if P_mech > 0 && P_elec > 0
    fprintf('  Efficiency    = %7.2f %%\n', (P_mech/P_elec)*100);
else
    fprintf('  Efficiency    = NaN (P_mech=%.3f, P_elec=%.3f)\n', P_mech, P_elec);
end

function simIn = buildSimIn_debug(modelName, mc, T_cmd, w_cmd, fsw_val)
    f_c = fsw_val/10;
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimulationMode',  'rapid-accelerator');
    simIn = simIn.setModelParameter('SimscapeLogType', 'all');
    simIn = simIn.setModelParameter('SimscapeLogName', 'simlog');
    simIn = simIn.setVariable('lambda',  mc.lambda);
    simIn = simIn.setVariable('p',       mc.p);
    simIn = simIn.setVariable('L_d',     mc.L_d);
    simIn = simIn.setVariable('L_q',     mc.L_q);
    simIn = simIn.setVariable('R',       mc.R);
    simIn = simIn.setVariable('V_bat',   mc.V_bat);
    simIn = simIn.setVariable('T_e_ref', T_cmd);
    simIn = simIn.setVariable('w_ref',   w_cmd);
    simIn = simIn.setVariable('fsw',     fsw_val);
    simIn = simIn.setVariable('Kp_d',    mc.L_d * 2*pi*f_c);
    simIn = simIn.setVariable('Ki_d',    mc.R   * 2*pi*f_c);
    simIn = simIn.setVariable('Kp_q',    mc.L_q * 2*pi*f_c);
    simIn = simIn.setVariable('Ki_q',    mc.R   * 2*pi*f_c);
    simIn = simIn.setBlockParameter([modelName '/PMSM'], 'angular_velocity_priority', 'low');
end
