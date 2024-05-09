%% Mobile Robot Explicit Model Predictive Controller
% L Parrish May 6, 2024
clear; clc; close all;

%% Description

% Goal: Simulate precomputed MPC policy for a trajectory following problem
% Process:  - Define a parametric curve for the controller to follow
%           - Load precomputed control policy
%           - Simulate Robot dynamics following trajectory

%% Load Control Policy
try
    load("MPCPolicy.mat");
catch
    error("Unable to find control policy .mat file");
end

assert(isvarname("w_l_opt"));
assert(isvarname("w_r_opt"));

%% Define Parametric curve
tF = 40;
Ts = 0.1;
t = 0:Ts:tF;
nx = 3;
nu = 2;
xref = zeros(nx,length(t));
xref(1,:) = 4*sin(t/7).*(1+cos(t/7));
xref(2,:) = 4*sin(t/7).*(1-cos(t/7));
for i = 1:length(t)-1
    xref(3,i) = atan2(xref(2,i+1) - xref(2,i),xref(1,i+1) - xref(1,i));
end
xref(3,length(t)) = xref(3,length(t)-1);

%% Simulate MPC Control Policy
[X,Y,Z] = ndgrid(xArray,yArray,tArray);
x = xref(:,1);
xHistory = x;
uHistory = [0; 0];
tic;
for k = 1:length(t)
    xHistory = [xHistory x];
    w_l = interpn(X,Y,Z,w_l_opt,x(1) - xref(1,k),x(2) - xref(2,k),wrapToPi(x(3)),"linear");
    w_r = interpn(X,Y,Z,w_r_opt,x(1) - xref(1,k),x(2) - xref(2,k),wrapToPi(x(3)),"linear");
    u = [w_l; w_r];
    uHistory = [uHistory u];
    x = UnicycleDiscrete(x,u,Ts);
    x(3) = wrapToPi(x(3));

end
disp("Average MPC Iteration Time (ms): ");
disp(toc/k * 1000);
xHistory(:,1) = [];
uHistory(:,1) = [];

%% Plot Results
figure;
Ylabels = {'x [m]','y [m]','\theta [rad]'};
for p = 1:nx
    subplot(nx,1,p);
    hold on;
    plot(t,xHistory(p,:));
    plot(t,xref(p,1:length(t)),'r--');
    xlabel('Time [s]');
    ylabel(Ylabels(p));
    legend('State Variable','Reference');
end
sgtitle("State Variables");


figure;
Ylabels = {'\omega_{L} [rad/s]','\omega_{R} [rad/s]'};
for p = 1:nu
    subplot(nu,1,p);
    stairs(t,uHistory(p,:));
    xlabel('Time [s]');
    ylabel(Ylabels(p));
end
sgtitle("Actuator Inputs");

figure;
hold on;
grid on;
plot(xHistory(1,:),xHistory(2,:));
plot(xref(1,1:length(t)),xref(2,1:length(t)),'r--');
xlabel('x [m]');
ylabel('y [m]');
title('Robot Trajectory');
legend('Actual Trajectory','Reference Trajectory');