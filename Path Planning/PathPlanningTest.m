%Create State Space
ss = stateSpaceSE2;

%Create an occupancy Map based state validator using the created state spce
sv = validatorOccupancyMap(ss);

%Create an occupancy map from an example map and set map resolution as 10
%cells/meter
load occupancygrid.mat map

sv.Map = map;

%Set validation distance for the validator
sv.ValidationDistance = 0.01;

%Update state space bounds to be the same as map limits
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];

%Create the path planner and incrase th maximum connection distance
planner = plannerRRT(ss,sv,MaxConnectionDistance=0.3);

% SET THE START AND GOAL STATES
start = [0.5 2.25 0];
goal = [5 5 0];

%Plan a path with devault settings
rng(100,'twister'); % for repeatable result
[pthObj,solnInfo] = plan(planner,start,goal);

%Visualize the results
show(map)
hold on
%Tree expansion
%plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)