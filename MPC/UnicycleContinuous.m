function z_dot = UnicycleContinuous(z,u)
% States
x = z(1);       % Vehicle X Position    [m]
y = z(2);       % Vehicle Y Position    [m]
theta = z(3);   % Vehicle Heading       [rad]

% Inputs
v = u(1);   % Vehicle Linear Velocity   [m/s]
w = u(2);   % Vehicle Angular Velocity  [rad/s]

% Continuous Dynamics
z_dot = [   v*cos(theta);...
            v*sin(theta);...
            w];
end

