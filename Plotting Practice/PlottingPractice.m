clc; close all; clear;
%%

data = [1,2,3,4,5,6,7];
time = [1,2,3,4,5,6,7];
figure;
hold on;
grid on;

for i = 1:1:length(data)
    tic;
    plotter(data,time,i)
    disp(toc)
    pause(1-toc)
end

function plotter(data, time,j)
    plot(time(j),data(j), 'o')    
end