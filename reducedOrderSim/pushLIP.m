function xdot = pushLIP(x,u,pushForce,modelProperties)
% generate model of VHIP for MATLABS nonlinear MPC
% in the form of xdot = f(x,u);

% model parameters
g = 9.81;           % gravitational Acceleration

% extract parameters passed through mpc 
m = modelProperties(1);
z0 = modelProperties(3);

% Model Dynamics
xdot = zeros(2,1);  % initlize output 

xdot(1) = x(2);                                         % d/dt x = xdot                                       % d/dt z = zdot
xdot(2) = (g)/z0*(x(1)-u(1)) + pushForce/m;           % d/dt xdot = xddot


end