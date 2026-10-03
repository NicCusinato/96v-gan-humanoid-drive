%% FOC Dashboard Integration - Simple Setup
% This script creates the FOC_Hardware_Deployment_Dashboard.slx model
% by copying the base FOC_Hardware_Deployment.slx model.
% 
% Remaining steps are manual (using Simulink GUI) - see FOC_DASHBOARD_SETUP_STEPS.md

clear all; close all; clc;

fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║            FOC Hardware Deployment Dashboard Setup            ║\n');
fprintf('║                    Simple File Copy Utility                   ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% Get current directory
current_dir = pwd;
fprintf('Working directory: %s\n\n', current_dir);

%% Initial: Close any open models that might conflict
try
    bdclose all;
    pause(0.5);
catch
end

%% Step 1: Check that source model exists
source_file = fullfile(current_dir, 'FOC_Hardware_Deployment.slx');
dest_file = fullfile(current_dir, 'FOC_Hardware_Deployment_Dashboard.slx');

fprintf('Step 1: Locating source model...\n');
if ~isfile(source_file)
    fprintf('✗ ERROR: Source model not found at:\n  %s\n', source_file);
    fprintf('\n  Make sure you are running this script from the MyFOC code folder.\n');
    return;
end
fprintf('✓ Found: FOC_Hardware_Deployment.slx\n\n');

%% Step 2: Check if destination already exists
fprintf('Step 2: Checking destination...\n');
if isfile(dest_file)
    fprintf('⚠ Destination file already exists - will overwrite\n');
    try
        delete(dest_file);
        fprintf('  Deleted existing file.\n\n');
    catch ME
        fprintf('✗ ERROR deleting existing file: %s\n', ME.message);
        fprintf('  (Try closing the model in Simulink and re-running)\n\n');
        return;
    end
else
    fprintf('✓ Destination is clear\n\n');
end

%% Step 3: Copy the model
fprintf('Step 3: Copying model file...\n');
try
    copyfile(source_file, dest_file);
    fprintf('✓ Successfully created: FOC_Hardware_Deployment_Dashboard.slx\n\n');
catch ME
    fprintf('✗ ERROR copying file: %s\n', ME.message);
    return;
end

%% Step 4: Display next steps
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    NEXT STEPS (MANUAL)                        ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

fprintf('✓ FOC_Hardware_Deployment_Dashboard.slx has been created.\n\n');

fprintf('Now you need to manually add the telemetry integration:\n');
fprintf('─────────────────────────────────────────────────────────────\n\n');

fprintf('1. OPEN the model in Simulink:\n');
fprintf('   >> open_system(''FOC_Hardware_Deployment_Dashboard'')\n\n');

fprintf('2. FOLLOW the detailed setup guide:\n');
fprintf('   📄 FOC_DASHBOARD_SETUP_STEPS.md\n\n');
fprintf('   This guide includes:\n');
fprintf('   • Step 2: Add Format_Telemetry_FOC MATLAB Function block\n');
fprintf('   • Step 3: Populate the block with telemetry code\n');
fprintf('   • Step 4: Wire 15 input signals (eQEP, ADCs, FOC internals)\n');
fprintf('   • Step 5: Connect output to serial\n');
fprintf('   • Step 6: Build and deploy to LAUNCHXL-F28069M\n');
fprintf('   • Step 7: Run dashboard visualization\n\n');

fprintf('3. QUICK REFERENCE available:\n');
fprintf('   📄 QUICK_REFERENCE_CARD.txt (one-page visual guide)\n\n');

fprintf('4. DETAILED TECHNICAL REFERENCE:\n');
fprintf('   📄 FOC_DASHBOARD_INTEGRATION_GUIDE.md\n');
fprintf('   📄 FOC_SIGNAL_EXTRACTION_REFERENCE.md\n\n');

fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║  All documentation files are in this folder. Start with Step  ║\n');
fprintf('║  2 of FOC_DASHBOARD_SETUP_STEPS.md to add the telemetry     ║\n');
fprintf('║  block and wire the signals.                                 ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

fprintf('Files created:\n');
fprintf('  ✓ FOC_Hardware_Deployment_Dashboard.slx (the model to edit)\n');
fprintf('  ✓ Enhanced_Telemetry_Dashboard_FOC.m (dashboard app - run after deployment)\n\n');

fprintf('Ready to proceed? Open Simulink and follow FOC_DASHBOARD_SETUP_STEPS.md\n');
