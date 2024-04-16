%% Populate Occupancy Grid
% Take data from ultrsonic sensors, create an occupancy grid to feed to
% path planning

% from insertRay() matlab example

map = binaryOccupancyMap(5,5,4);

while scanning

    % each time step
    pose = [5,5,0]; % updated every time step, comes from ESP32
    distance_to_object = [sensor1,sensor2,sensor3]; % sensor readings from each sensor
    angle_of_object = [pose(3)+(pi/6),pose(3),pose(3)-(pi/6)]; % pose +/- sensor angles, always radians
    maxrange = 20; % idk what this is
    scan = lidarScan(distance_to_object,angle_of_object); % creates lidar scan object
    insertRay(map,pose,scan,maxrange); % updates occupancy grid
    show(map)
    hold on

end

% for testing, step through the big array Ellie will send from a continuous
% scan

