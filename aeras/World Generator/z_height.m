clc; clear; close all;

%% Load the heightmap from a .tif file
filename = 'terrain_1.tif'; 
heightmap = imread(filename);

% Convert to double for calculations
heightmap = double(heightmap);

% Determine the range of Z values (elevation)
min_z = min(heightmap(:));  % Minimum elevation
max_z = max(heightmap(:));  % Maximum elevation
z_range = max_z - min_z;    % Elevation range

fprintf('Minimum elevation: %.2f meters\n', min_z);
fprintf('Maximum elevation: %.2f meters\n', max_z);
fprintf('Recommended Z size for Gazebo: %.2f meters\n', z_range);
