function xdot = stateLIPPFW(x,u,modelProperties,~,~,~,~)
% generate model of LIPPFW for MATLABS nonlinear MPC
% in the form of xdot = f(x,u);

% model parameters
g = 9.81;           % gravitational Acceleration

% extract parameters passed through mpc 
m = modelProperties(1);     % model mass
J = modelProperties(2);     % flywheel inertia
z0 = modelProperties(3);    % rest height

% Model Dynamics
xdot = zeros(4,1);  % initlize output 

xdot(1) = x(3);                                         % d/dt x = xdot
xdot(2) = x(4);                                         % d/dt theta = thetadot
xdot(3) = (g/z0)*(x(1)-u(1)) - u(2)/(m*z0);             % d/dt xdot = xddot
xdot(4) = u(2)/J;                                       % d/dt thetadot = thetaddot

end