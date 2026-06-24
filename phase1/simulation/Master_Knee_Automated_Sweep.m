% =========================================================================
%  MATLAB Script: Generate Native PSIM Script (No Auto-Execution)
%  Target: 0 to Max Speed (458 RPM) & 0 to Max Torque (10 Nm Motor Shaft)
% =========================================================================
clear; clc;

% Arrays
Vdc_array    = [48.0, 96.0];
fsw_array    = [40000.0, 100000.0, 150000.0];
T_load_array = linspace(0.0, 10.0, 6);
nm_ref_array = linspace(0.0, 458.0, 6);

% Open Master Script File explicitly as a .script for the PSIM GUI
cmd_filename = fullfile(pwd(), 'Master_Knee_Sweep.script');
fid = fopen(cmd_filename, 'w', 'n', 'US-ASCII');

fprintf(fid, '// PSIM Master Execution Script\n');
fprintf(fid, '// This script uses native Simulate() functions to execute the schematic.\n\n');

for Vdc = Vdc_array
    for fsw = fsw_array
        
        fsam = fsw / 2.0; fcr_i = fsam / 10.0; fcr_w = fcr_i / 10.0;
        
        T_d = 0.00015 / 0.15;            K_d = 2 * pi * fcr_i * 0.00015;
        T_q = 0.00022 / 0.15;            K_q = 2 * pi * fcr_i * 0.00022;
        K_w = (2 * pi * fcr_w * 0.00555) / 0.511;
        
        if fcr_w > 0
            T_w = 1.0 / (2 * pi * (fcr_w / 10.0));
        else
            T_w = 999.0; 
        end
        
        for T_load = T_load_array
            for nm_ref = nm_ref_array
                
                base_name = sprintf('KneeRun_V%.0f_F%.0f_T%.1f_N%.1f', Vdc, fsw, T_load, nm_ref);
                output_file = [base_name, '.txt']; 
                
                % Write pure variable assignments (No 'set' keyword)
                fprintf(fid, 'Vdc = %.1f; fsw = %.1f; T_load = %.2f; nm_ref = %.1f;\n', Vdc, fsw, T_load, nm_ref);
                fprintf(fid, 'K_d = %.6f; T_d = %.6f;\n', K_d, T_d);
                fprintf(fid, 'K_q = %.6f; T_q = %.6f;\n', K_q, T_q);
                fprintf(fid, 'K_w = %.6f; T_w = %.6f;\n', K_w, T_w);
                
                % The core execution function: Simulate(Schematic, OutputFile)
                fprintf(fid, 'Simulate("HumanoidSimV2.psimsch", "%s");\n\n', output_file);
            end
        end
    end
end

fclose(fid);
disp('Master script generated! Open PSIM Script Tool and run Master_Knee_Sweep.script');