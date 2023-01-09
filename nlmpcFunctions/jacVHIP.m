function [A,B]  = jacVHIP(x,u,~,~,~,~,~,~)
% generate jacobians for VHIP model used in MATLABS nonlinear MPC
% A(i,j) = del/del x(j) (xdot(i))
% B(i,j) = del/del u(j) (xdot(i))


% model parameters
g = 9.81;       % gravitational Acceleration

% extract parameters passed through mpc 


% analytical jacobian with respect to state variables
A = zeros(4);

A(1,3) = 1;
A(2,4) = 1;
A(3,1) = (g+u(2))/x(2);
A(3,2) = -((g+u(2))*(x(1)-u(1)))*x(2)^(-2);


% analytical jacobian with respect to the manipulated variables
B = zeros(4,2);

B(3,1) = -(g+u(2))/x(2);
B(3,2) = (x(1)-u(1))/x(2);
B(4,2) = 1;

end