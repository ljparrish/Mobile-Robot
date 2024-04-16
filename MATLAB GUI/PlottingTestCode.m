clc; clear; close all;
%% 
xPos = 20;
yPos = 10;
theta = pi/2;

robotWidth = 0.2; % Change values
robotLength = 0.2; % Change values
[newX, newY] = GenerateRobot(xPos, yPos, robotLength, robotWidth, theta);
fill(newX, newY, 'r', 'LineStyle','none');

  


