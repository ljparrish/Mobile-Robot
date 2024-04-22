%% Populate Occupancy Grid
% Take data from ultrsonic sensors, create an occupancy grid to feed to
% path planning

% from insertRay() matlab example

close all
clear all
clc

% read in CSVs
readings_all = readtable('Book1.csv');
readings_all = table2array(readings_all);
readings_all = 0.01*readings_all; % in meters

angles_all = linspace(pi,0,length(readings_all)/3);
angles_1 = angles_all + ((pi/6)*ones(length(angles_all,1)));
angles_2 = angles_all;
angles_3 = angles_all(n) - ((pi/6)*ones(length(angles_all,1)));

readings_1 = {};
readings_2 = {};
readings_3 = {};

for n = 1:3:length(readings_all)
    readings_1 = [readings_1 readings_all(n)];
end

for k = 2:3:length(readings_all)
    readings_2 = [readings_1 readings_all(n)];
end

for j = 3:3:length(readings_all)
    readings_3 = [readings_1 readings_all(n)];
end

map = binaryOccupancyMap(5,5,4);
% each time step
pose = [2.5,0,0]; % updated every time step, comes from ESP32
% distance_to_object = [sensor1,sensor2,sensor3]; % sensor readings from each sensor
%distance_test = 3*ones(22,1);
%angles = linspace(pi,0,length(ultrasonic_readings));

readings_final = vertcat(readings_1,readings_2,readings_3);
angles_final = vertcat(angles_1,angles_2,angles_3);

%angles = linspace(pi,0,length(readings));
% angle_of_object = [pose(3)+(pi/6),pose(3),pose(3)-(pi/6)]; % pose +/- sensor angles, always radians
maxrange = 20; % idk what this is
scan = lidarScan(readings_final,angles_final); % creates lidar scan object
insertRay(map,pose,scan,maxrange); % updates occupancy grid
show(map)
hold on

A = [];
% for testing, step through the big array Ellie will send from a continuous
% scan

