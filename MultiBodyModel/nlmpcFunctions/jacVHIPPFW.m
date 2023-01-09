function [A,B]  = jacVHIPPFW(x,u,modelProperties,~,~,~,~)
% generate jacobians for VHIPPFW model used in MATLABS nonlinear MPC
% A(i,j) = del/del x(j) (xdot(i))
% B(i,j) = del/del u(j) (xdot(i))


% model parameters
g = 9.81;       % gravitational Acceleration

% extract parameters passed through mpc 
m = modelProperties(1);     % model mass
J = modelProperties(2);     % flywheel inertia


% analytical jacobian with respect to state variables
A = zeros(6);

A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,1) = (g+u(3))/x(3);
A(4,3) = -((g+u(3))*(x(1)-u(1)) - u(2)/m )*x(3)^(-2);


% analytical jacobian with respect to the manipulated variables
B = zeros(6,3);

B(4,1) = -(g+u(3))/x(3);
B(4,2) = -1/(m*x(3));
B(4,3) = (x(1)-u(1))/x(3);
B(5,2) = 1/J;
B(6,3) = 1;

end