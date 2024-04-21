%% Serial Com Practice
close all 
clear all
clc

% % create ESP32 Object
% espObj = arduino("COM3", "WALE Robot", "Libraries", {'Serial'});
% % create serial object
% serialdevObj = device(espObj,'COM3',1,'BaudRate', 115200);
% 
% % write 3 bytes of data to the serial device
% write(serialdevObj, [88 99 45]);

% read data from the serial device
read(serialdevObj, 3)

comChannel = serialportlist;

s = serialport(comChannel(6),115200);

write(s, [0,0], "int8")

r = read(s,36,"int8");
disp(r)
r = r(1:6); %extracting important values

rightWheelPulse = typecast(r(1),'int8');
leftWheelPulse = typecast(r(2),'int8');
count = typecast(r(6),'int8');
disp(rightWheel)
disp(leftWheel)




