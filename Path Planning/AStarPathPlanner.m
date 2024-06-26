close all; clc; clear;
%% A* Path Planning
%Load occupancy grid
load occupancygrid.mat map

%Create State Space
ss = stateSpaceSE2;

% Update state space bounds to be the same as map limits.
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];

% Create state validator object to check for collisions
sv = validatorOccupancyMap(ss);

% Assign the map to the state validator object
sv.Map = map;

%% Plan and visualize path
% Initialize the planner and specify values for Minimum turning radius and
% motion primitive length. Getting rid of either of those drastically 
% increases computation time
planner = plannerHybridAStar(sv, ...
                             MinTurningRadius=0.15, ...
                             MotionPrimitiveLength=0.2);

% Define start and goal poses for the vehicle as [x, y, theta] vectors. x 
% and y specify the position in meters, and theta specifies the orientation
% angle in radians.

startPose = [2 1 pi/2]; % [meters, meters, radians]
% show(map)
% pause(1);
% h=msgbox('Please Select the Target using the Left Mouse button');
% uiwait(h,5);
% if ishandle(h) == 1
%     delete(h);
% end
% xlabel('Please Select the Target using the Left Mouse button','Color','black');
% but=0;
% while (but ~= 1) %Repeat until the Left button is not clicked
%     [xval,yval,but]=ginput(1);
% end
% xval=floor(xval);
% yval=floor(yval);
% goalOrientation = -pi/2;
% goalPose = [xval yval goalOrientation];
goalPose = [8 8 2*pi];

%Plan a path from the start pose to the goal pose.
tic;
refpath = plan(planner,startPose,goalPose,SearchMode='greedy');     
%disp(toc);

% Extract the path poses from the path object
points = refpath.States;
points = points';
xValues = points(1,:);
yValues = points(2,:);
totalDistance = 0;
for i = 1:1:length(xValues)-1
    xDiff = xValues(i+1)-xValues(i);
    tempDistance = sqrt(((xValues(i+1)-xValues(i))^2)+((yValues(i+1)-yValues(i))^2));
    totalDistance = totalDistance + tempDistance;
end
robotAvgSpeed = 0.5;
endTimePoint = totalDistance/robotAvgSpeed;
disp(endTimePoint)
timePoints = [0 endTimePoint];
timeVector = 0:0.1:endTimePoint;

[q, qd, qdd, pp] = bsplinepolytraj(points,timePoints,timeVector);
%Visualize the path using show function.

%show(planner)

save AStarTrajecotory.mat q




