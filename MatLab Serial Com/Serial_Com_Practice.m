%% Serial Com Practice
close all 
clear all
clc

% create ESP32 Object
espObj = arduino("COM4", "WALE Robot", "Libraries", {'I2C','Serial','SPI'});
% create serial object
serialdevObj = device(espObj,'COM4',1,'BaudRate', 115200);

% write 3 bytes of data to the serial device
write(serialdevObj, [88 99 45]);

% read data from the serial device
read(serialdevObj, 3)

