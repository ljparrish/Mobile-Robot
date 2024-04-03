%% Serial Com Practice
close all 
clear all
clc

% create serial object
serialdevObj = device(arduinoObj,'COM4',1,'BaudRate', 115200);

