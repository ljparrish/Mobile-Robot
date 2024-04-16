clc; clear; close all;
%% 
xPos = 20;
yPos = 10;
theta = pi/4;
theta = rad2deg(theta);

robotWidth = 10; % Change values
robotLength = 5; % Change values

for i = 1:1:length(xPos)

    GenerateRobot(xPos(i), yPos(i), robotLength, robotWidth, 'r', theta)
end
  


