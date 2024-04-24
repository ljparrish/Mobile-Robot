close all
clear all
clc

% Creates occupancy map with width and height in units of meters
% resolution of 10 means 10 cells per meter
map = binaryOccupancyMap(5,5,10);

% assuming starting pose is (1,2.5,0)

% Creating a numeric array with matching size
walls = zeros(50,50);
walls(1,:) = 1; % Top wall
walls(end,:)= 1; % Bottom wall
walls(:,1) = 1; % Left wall
walls(:,end) = 1; % Right wall

% walls(1:25,7) = 1; % Left division
walls(1:15,35)= 1; % Middle division
walls(15,35:end) = 1;

%walls(1:25,32) = 1;
% walls(1:15,30) = 1; %Right Division

% Set occupancy of world locations and show map


setOccupancy(map, [1,1], walls, "grid")

%Inflate the thickness of our occupied spaces to give more room for robot
%to navigate around obstacles
inflate(map, 0.1)
figure
show(map)

save impossibleGrid.mat map

% %%  Create occupancy map with cost values
% costValue = zeros(10,10);
% costValue(1,:) = 1; % Top wall
% costValue(end,:)= 1; % Bottom wall
% costValue(:,1) = 1; % Left wall
% costValue(:,end) = 1; % Right wall
% costValue(1:3,4) = 1; % Left division
% costValue(2:end, 4)= 1; % Middle division
% costValue(1:3,6) = 1; %Right Division
% 
% costValuesMap = binaryOccupancyMap(costValue, 1);
% show(costValuesMap)
% 
% 
% 
% save costvalueoccupancygrid.mat map
