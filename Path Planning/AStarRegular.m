close all; clc; clear;

%% Load Occupancy Grid 
load occupancygrid.mat map

%set start and end positions
startPosition = [2 2];

% Create A* planner object
planner = plannerAStarGrid(map);

show(map)
pause(1);
h=msgbox('Please Select the Target using the Left Mouse button');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('Please Select the Target using the Left Mouse button','Color','black');
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
end
xval=floor(xval);
yval=floor(yval);
goalPosition = [xval yval];

plan(planner,startPosition,goalPosition);


show(planner)


