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
fig1 = map;
figure(1)
show(map)

inflate(fig1, 0.1);

figure(2)
show(fig1)

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

%% Remove invalid object test
clear all
occupancyGrid = binaryOccupancyMap(5,5,10);

pose = [1 2.5 0]; % in m and radians
angle_range_1 = linspace((pi()/6)-(pi()/18),(pi()/6)+(pi()/18),10);
angle_range_2 = linspace(-(pi()/18),(pi()/18),10);
angle_range_3 = linspace(-(pi()/6)-(pi()/18),-(pi()/6)+(pi()/18),10);
readings_1 = ones(1,10)*150*0.01;
readings_2 = ones(1,10)*145*0.01;
readings_3 = ones(1,10)*150*0.01;
angles = [angle_range_1,angle_range_2,angle_range_3];
readings = [readings_1,readings_2,readings_3]; % in m
maxrange = 500;
 
scan = removeInvalidData(lidarScan(readings,angles),'RangeLimits',[0.01 1.25]); % creates lidar scan object
insertRay(occupancyGrid,pose,scan,maxrange); % updates occupancy grid

show(occupancyGrid);
