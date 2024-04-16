clc; clear; close all;
%% 
xPos = [20, 20, 20, 20, 20, 20, 20, 20, 25];
yPos = [10, 15, 20, 25, 30, 35, 40, 45, 55];
robotWidth = 10; % Change values
robotLength = 5; % Change values

for i = 1:1:length(xPos)

    X = xPos(i) - (robotWidth/2);
    Y = yPos(i) - (robotLength/2);
    pos = [X, Y, robotWidth, robotLength];
    rectangle('Position', pos,'FaceColor',[0 0 0],'EdgeColor','r')
end
  


