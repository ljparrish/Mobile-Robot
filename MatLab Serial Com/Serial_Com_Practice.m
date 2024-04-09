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
% 
% % read data from the serial device
% read(serialdevObj, 3)

comChannel = serialportlist;

s = serialport(comChannel(1),9600);
data = [88, 99, 45];
write(s, data, "int16")

r = read(s,256,"int16");

disp(r)
