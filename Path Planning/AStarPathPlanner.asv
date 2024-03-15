%% A* Path Planning
%Load occupancy grid
load costvalueoccupancygrid.mat map

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
                             MinTurningRadius=0.75, ...
                             MotionPrimitiveLength=0.5);

% Define start and goal poses for the vehicle as [x, y, theta] vectors. x 
% and y specify the position in meters, and theta specifies the orientation
% angle in radians.

startPose = [2 1 pi/2]; % [meters, meters, radians]
goalPose = [8 8 2*pi];

%Plan a path from the start pose to the goal pose.
refpath = plan(planner,startPose,goalPose,SearchMode='exhaustive');     

% Extract the path poses from the path object
path = refpath.States;

%Visualize the path using show function.

show(planner)

save AStarTrajecotory.mat path




