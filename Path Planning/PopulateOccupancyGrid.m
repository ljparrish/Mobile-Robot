%% Populate Occupancy Grid
% Take data from ultrsonic sensors, create an occupancy grid to feed to
% path planning

close all
clear all
clc

% Read distance data from CSV
readings_all = readtable('Book1.csv');
readings_all = table2array(readings_all);
readings_all = 0.01 * readings_all; % in meters

% Dummy angle of rotating pi for testing purposes
% Testing scenario: rotate for pi (occupied by wall -> free -> occupied by
% wall)
size_reading = floor(length(readings_all)/3);
angles = linspace(0, pi, size_reading);
angles = angles';

% Initialize map
map = binaryOccupancyMap(5,5,20);
maxrange = 20;

% Each time step
pose = [2.5, 0, pi];     % updated every time step, comes from ESP32

% Testing one sensor

% sensor 1
% scan = lidarScan(readings_all(1:3:length(readings_all)), -angles+(pi/6));
% insertRay(map, pose, scan, maxrange);

% sensor 2
% scan = lidarScan(readings_all(2:3:length(readings_all)), -angles);
% insertRay(map, pose, scan, maxrange); 

% sensor 3
% scan = lidarScan(readings_all(3:3:length(readings_all)), -angles-(pi/6));
% insertRay(map, pose, scan, maxrange);

% Testing three sensors
for i=1:size_reading
    range = [readings_all(1+(i-1)*3), readings_all(2+(i-1)*3), readings_all(3+(i-1)*3)];
    angle = [-angles(i)+(pi/6), -angles(i), -angles(i)-(pi/6)];
    % angle_of_object = [pose(3)+(pi/6),pose(3),pose(3)-(pi/6)];            % pose +/- sensor angles, always radians
    scan = lidarScan(range, angle);                                         % creates lidar scan object
    insertRay(map, pose, scan, maxrange);                                   % updates occupancy grid
end

show(map)
hold on