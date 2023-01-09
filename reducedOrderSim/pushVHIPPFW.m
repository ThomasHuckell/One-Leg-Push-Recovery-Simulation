function xdot = pushVHIPPFW(x,u,pushForce,modelProperties)
% generate model of VHIPPFW for MATLABS nonlinear MPC
% in the form of xdot = f(x,u);

% model parameters
g = 9.81;           % gravitational Acceleration

% extract parameters passed through mpc 
m = modelProperties(1);     % model mass
J = modelProperties(2);     % flywheel inertia


% Model Dynamics
xdot = zeros(6,1);  % initlize output 

xdot(1) = x(4);                                         % d/dt x = xdot
xdot(2) = x(5);                                         % d/dt theta = thetadot
xdot(3) = x(6);                                         % d/dt z = zdot
xdot(4) = (g+u(3))/x(3)*(x(1)-u(1)) - u(2)/(m*x(3)) + pushForce/m;    % d/dt xdot = xddot
xdot(5) = u(2)/J;                                       % d/dt thetadot = thetaddot
xdot(6) = u(3);                                         % d/dt zdot = zddot


end