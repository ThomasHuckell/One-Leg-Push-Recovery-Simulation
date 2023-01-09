function xdot = stateVHIP(x,u,~,~,~,~,~)
% generate model of VHIP for MATLABS nonlinear MPC
% in the form of xdot = f(x,u);

% model parameters
g = 9.81;           % gravitational Acceleration

% extract parameters passed through mpc 

% Model Dynamics
xdot = zeros(4,1);  % initlize output 

xdot(1) = x(3);                                         % d/dt x = xdot
xdot(2) = x(4);                                         % d/dt z = zdot
xdot(3) = (g+u(2))/x(2)*(x(1)-u(1));                    % d/dt xdot = xddot
xdot(4) = u(2);                                         % d/dt zdot = zddot


end