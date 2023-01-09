function [G, Gmv, Ge] = costJacobianLIP(X,U,~,data,~,stateLim,~,stateWeight,controlWeight)
% generate analytical jacobians for the custom cost function

% model parameters extracted from passed parameters
delta = stateLim(1);
xdotMax = stateLim(4);

% extract mpc details
p = data.PredictionHorizon;
Nx = data.NumOfStates;
Nmv = length(data.MVIndex);

G = zeros(p,Nx); 

G(1:p,1) = -2*stateWeight(1)*delta^(-2)*(data.References(1:p,1)-X(2:p+1,1));
G(1:p,2) = -2*stateWeight(4)*xdotMax^(-2)*(data.References(1:p,2)-X(2:p+1,2));

Gmv = zeros(p,Nmv);

Gmv(1:p,1) = 2*controlWeight(1)*delta^(-2)*U(1:p,1);

Ge = 0;

end