%% Mobile Robot Explicit Model Predictive Controller
% L Parrish May 6, 2024
clear; clc; close all;
format short;
format compact;

%% Description

% Goal: Compute an MPC policy for the mobile robot using a grid approach
% Process:  - Discretize the space of errors in the three system states
%           - Compute optimal control inputs for each point in the error space
%           - Put control policy into 3D array for interpolation
%           - Save to .mat file for other scripts

%% MPC Formulation
% Set Number of States (nx), number of outputs (ny), and inputs or Manipulated Variables (nu)
nx = 3;
ny = 3; % Assuming full state feedback
nu = 2;

% Create a non-linear MPC controller
MPCController = nlmpc(nx,ny,nu);

% Set Discrete Sampling Time in seconds
Ts = 0.1;
MPCController.Ts = Ts;

% Set Prediction Horizon and Control Horizon
MPCController.PredictionHorizon = 10;
MPCController.ControlHorizon = 5;

% Set the discrete time system model for Unicycle Robot
MPCController.Model.StateFcn = "UnicycleDiscrete";
MPCController.Model.IsContinuousTime = false;
MPCController.Model.NumberOfParameters = 1;

% Set the system output model for the unicycle robot
MPCController.Model.OutputFcn = "UnicycleOutput";

% Define MPC Cost
MPCController.Weights.OutputVariables = [10 10 0]; % State Error Cost
MPCController.Weights.ManipulatedVariables = [0.1 0.1]; % Input Cost
MPCController.Weights.ManipulatedVariablesRate = [0.1 0.1]; % Input Rate Cost

% Define Constraint on State Variables
MPCController.OV(1).Min = -10;
MPCController.OV(1).Max = 10;

MPCController.OV(2).Min = -10;
MPCController.OV(2).Max = 10;

% Define Constraints on Actuator Inputs
MPCController.MV(1).Min = -20;
MPCController.MV(1).Max = 40;

MPCController.MV(2).Min = -20;
MPCController.MV(2).Max = 40;

% Define Constraints on Actuator Input Rate of Change
MPCController.MV(1).RateMin = -40;
MPCController.MV(1).RateMax = 40;

MPCController.MV(2).RateMin = -40;
MPCController.MV(2).RateMax = 40;

options = nlmpcmoveopt;
options.Parameters = {Ts};

%% Create Error Space Grid
numGridPoints = 15; % Set the number of grid points for each state dimension
xBounds = [-2 2];
yBounds = [-2 2];
tBounds = [-pi() pi()];

xArray = linspace(xBounds(1),xBounds(2),numGridPoints);
yArray = linspace(yBounds(1),yBounds(2),numGridPoints);
tArray = linspace(tBounds(1),tBounds(2),numGridPoints);

%% Initialize Optimal Control Policy Matrix
w_r_opt = zeros(numGridPoints,numGridPoints,numGridPoints);
w_l_opt = zeros(numGridPoints,numGridPoints,numGridPoints);

%% Main Nested Loop
tic;
x0 = [0; 0; 0];
u0 = [0; 0];
progressBar = waitbar(0,'Computing Policy . . . 0%');
counter = 0;
for xidx = 1:length(xArray)
    for yidx = 1:length(yArray)
        for tidx = 1:length(tArray)
            e = [xArray(xidx) yArray(yidx) tArray(tidx)];
            u_opt = nlmpcmove(MPCController,e,u0,[0 0 0],[],options);
            w_l_opt(xidx,yidx,tidx) = u_opt(1);
            w_r_opt(xidx,yidx,tidx) = u_opt(2);
            counter = counter + 1;
            waitbar(counter/numel(w_r_opt),progressBar,['Computing Policy . . . ',sprintf('%d',round(counter/numel(w_r_opt)*100)),'%']);
        end
    end
end
close(progressBar);
disp(toc);

%% Save to .mat
save("MPCPolicy.mat","w_r_opt","w_l_opt","xArray","yArray","tArray");
