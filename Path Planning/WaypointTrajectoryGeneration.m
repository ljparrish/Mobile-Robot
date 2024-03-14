
%% Waypoint generation
%Load occupancy grid
load occupancygrid.mat map

%set start and end positions
startPosition = [1 1];
goalPosition = [8 8];

%Create a mobileRobotPRM object with a binary occupancy map and specify the
% maximum number of nodes. Specify the maximum distance between the two
% connected notes

numnodes = 2000;
planner = mobileRobotPRM(map, numnodes);
planner.ConnectionDistance = 1;

%Find a path between the start and goal positions and generates waypoints
%in between
waypoints = findpath(planner,startPosition, goalPosition);

disp(waypoints)

%% Trajectory Generation
% Robot height from base.
robotheight = 0.12;
% Number of waypoints
numWaypoints = size(waypoints,1);
%Robot arrival time at first waypoint
firstInTime = 0;
% Robot arrival time at last waypoint
lastInTime = firstInTime + (numWaypoints-1);
% Generate waypoint trajectory with waypoints from planned path
traj = waypointTrajectory(SampleRate=10,...
    TimeOfArrival=firstInTime:lastInTime,...
    Waypoints=[waypoints, robotheight*ones(numWaypoints,1)],...
    ReferenceFrame="ENU");

