% Creates occupancy map with width and height in units of meters
% resolution of 10 means 10 cells per meter
map = binaryOccupancyMap(10,10,10);

% Creating a numeric array with matching size
walls = zeros(100,100);
walls(1,:) = 1; % Top wall
walls(end,:)= 1; % Bottom wall
walls(:,1) = 1; % Left wall
walls(:,end) = 1; % Right wall
walls(1:50,15) = 1; % Left division
walls(25:end, 40)= 1; % Middle division
%walls(1:30,35) = 1; %Right Division

% Set occupancy of world locations and show map


setOccupancy(map, [1,1], walls, "grid")

%Inflate the thickness of our occupied spaces
inflate(map, 0.15)
figure
show(map)

save occupancygrid.mat map