
%% Waypoint generation
%Load occupancy grid
load occupancygrid.mat map

%set start and end positions
startPosition = [1 1];
goalPosition = [8 8];

%Create a mobileRobotPRM object with a binary occupancy map and specify the
% maximum number of nodes. Specify the maximum distance between the two
% connected notes

% Increasing this number decreases noise in our trajectory planning but will increase computation time
numnodes = 2000;
planner = mobileRobotPRM(map, numnodes);

% Adjust this value to create more or less control points for Bezier.
% Smaller value for more points, greater value for less points
planner.ConnectionDistance = 0.5; 

%Find a path between the start and goal positions and generates waypoints
%in between
waypoints = findpath(planner,startPosition, goalPosition);

waypoints = transpose(waypoints);
disp(waypoints)
%show(map)
%hold on 
%plot(waypoints(:,1),waypoints(:,2))

%% B-Spline Curve Generation

timePoints = [0 50];
timeVector = 0:0.01:50;

[q, qd, qdd, pp] = bsplinepolytraj(waypoints,timePoints,timeVector);

figure
show(map)
hold on
plot(waypoints(1,:),waypoints(2,:),'xb-')
hold all
plot(q(1,:), q(2,:))
xlabel('X')
ylabel('Y')
hold off

save BezierTrajectory.mat q

