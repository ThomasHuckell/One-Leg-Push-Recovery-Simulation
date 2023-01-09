function xdot = stateLIP(x,u,modelProperties,~,~,~,~)
% generate model of LIP for MATLABS nonlinear MPC
% in the form of xdot = f(x,u);

% model parameters
g = 9.81;           % gravitational Acceleration

% extract parameters passed through mpc 
z0 = modelProperties(3);     % model mass



% Model Dynamics
xdot = zeros(2,1);  % initlize output 

xdot(1) = x(2);                                         % d/dt x = xdot
xdot(2) = (g/z0)*(x(1)-u(1)) ;                          % d/dt xdot = xddot


end