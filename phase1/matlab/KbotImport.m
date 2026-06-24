clear; close all;
% Path to your URDF file
urdfFile = fullfile('C:\96v_gan_humanoid_drive\phase1\matlab\urdf', 'robot_legs.urdf');

% Import into Simscape Multibody
[mdlHandle, dataFileName] = smimport(urdfFile);

% Open the generated model
open_system(mdlHandle);

