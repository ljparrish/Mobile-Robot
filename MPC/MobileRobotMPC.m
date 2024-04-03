%% Mobile Robot Model Predictive Controller for Trajectory Following
% L Parrish March 10, 2024
clear; clc; close all;

%% MPC Controller Creation and Setup
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

% Provide the system Jacobian
% MPCController.Jacobian.OutputFcn = @(x,u,Ts) [1 0 -u(1)*sin(x(3)); 0 1 u(1)*sin(x(3)); 0 0 1];

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
MPCController.MV(1).RateMin = -5;
MPCController.MV(1).RateMax = 5;

MPCController.MV(2).RateMin = -5;
MPCController.MV(2).RateMax = 5;

%% MPC Controller Validation
x0 = [2; 1; pi/2];
u0 = [0; 0];
validateFcns(MPCController,x0,u0,[],{Ts});

%% Create or Load Reference Trajectory
try
    load("AStarTrajecotory.mat")
    q = q(:,1:4:end); % Downsample q
    xref = zeros(nx,size(q,2));
    q = q';
    
    xref = zeros(size(q,1),2);
    xref(:,1) = q(:,1);
    xref(:,2) = q(:,2);
    xref(:,3) = q(:,3);
    t = 0:Ts:Ts*(size(q,1)-1);
    %t = 0:Ts:Ts*(size(q,1)-1);
    t = t';
catch
    tF = 60;
    t = 0:Ts:tF;
    xref = zeros(nx,length(t));
    %xref(1,:) = 4*sin(t/7).*(1+cos(t/7));
    %xref(2,:) = 4*sin(t/7).*(1-cos(t/7));
end
    %xref = xref';

%% Simulate MPC Controller

options = nlmpcmoveopt;
options.Parameters = {Ts};
x = x0;
u = u0;
xHistory = x;
uHistory = u;
trajectorylookahead = 5;
xref = [xref; xref(end,:); xref(end,:); xref(end,:); xref(end,:); xref(end,:)];
[coreData, onlineData] = getCodeGenerationData(MPCController, x0, u0, {Ts});
tic;
for k = 1:length(t)
    % Update state reference
    onlineData.ref = xref(k:k+trajectorylookahead,:);

    % Compute MPC
    [u, onlineData] = nlmpcmoveCodeGeneration(coreData, x, u, onlineData);

    % Apply optimal inputs and simulate next timestep
    x = UnicycleDiscrete(x,u,Ts);

    % Save States and Input Histories
    xHistory = [xHistory x];
    uHistory = [uHistory u];
end
%% Codegen Block
codeGenEnabled = 0;
if(codeGenEnabled)
    func = 'nlmpcmoveCodeGeneration';
    funcOutput = 'nlmpcmoveMEX';
    Cfg = coder.config("lib");
    Cfg.TargetLang = 'C';
    Cfg.DynamicMemoryAllocation = 'off';
    codegen('-config',Cfg,func,'-o',funcOutput,'-args',{coder.Constant(coreData),x,u,onlineData});
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
    plot(t,xref(1:length(t),p),'r--');
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
plot(xref(1:length(t),1),xref(1:length(t),2),'r--');
xlabel('x [m]');
ylabel('y [m]');
title('Robot Trajectory');
legend('Actual Trajectory','Reference Trajectory');