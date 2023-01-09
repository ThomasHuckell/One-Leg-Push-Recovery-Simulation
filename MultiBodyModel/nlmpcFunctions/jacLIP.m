function [A,B]  = jacLIP(~,~,modelProperties,~,~,~,~)
% generate jacobians for LIP model used in MATLABS nonlinear MPC
% A(i,j) = del/del x(j) (xdot(i))
% B(i,j) = del/del u(j) (xdot(i))


% model parameters
g = 9.81;       % gravitational Acceleration

% extract parameters passed through mpc 
z0 = modelProperties(3);    % rest height

% analytical jacobian with respect to state variables
A = zeros(2);

A(1,2) = 1;
A(2,1) = (g/z0);


% analytical jacobian with respect to the manipulated variables
B = zeros(2,1);

B(2,1) = -(g/z0);

end