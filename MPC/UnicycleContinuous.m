function z_dot = UnicycleContinuous(z,u)
% Robot Constant Parameters
r_wheel = 0.030; % Wheel Radius [m]
d_wheel = 0.120; % Distance Between Wheels [m]

% States
x = z(1);       % Vehicle X Position    [m]
y = z(2);       % Vehicle Y Position    [m]
theta = z(3);   % Vehicle Heading       [rad]

% Inputs
w_l = u(1);     % Left Wheel Angular Velocity   [rad/s]
w_r = u(2);     % Left Wheel Angular Velocity   [rad/s]

% Continuous Dynamics
z_dot = [   (r_wheel/2*w_r + r_wheel/2*w_l)*cos(theta);...
            (r_wheel/2*w_r + r_wheel/2*w_l)*sin(theta);...
            r_wheel*w_r/d_wheel - r_wheel*w_l/d_wheel];
end

% V = r_wheel/2*w_r + r_wheel/2*w_l
% w = r_wheel*w_r/d_wheel - r_wheel*w_l/d_wheel