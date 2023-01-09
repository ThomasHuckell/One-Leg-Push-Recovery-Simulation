function [A,B]  = jacLIPPFW(~,~,modelProperties,~,~,~,~)
% generate jacobians for LIPPFW model used in MATLABS nonlinear MPC
% A(i,j) = del/del x(j) (xdot(i))
% B(i,j) = del/del u(j) (xdot(i))


% model parameters
g = 9.81;       % gravitational Acceleration

% extract parameters passed through mpc 
m = modelProperties(1);     % model mass
J = modelProperties(2);     % flywheel inertia
z0 = modelProperties(3);    % rest height

% analytical jacobian with respect to state variables
A = zeros(4);

A(1,3) = 1;
A(2,4) = 1;
A(3,1) = g/z0;


% analytical jacobian with respect to the manipulated variables
B = zeros(4,2);

B(3,1) = -g/z0;
B(3,2) = -1/(m*z0);
B(4,2) = 1/J;


end