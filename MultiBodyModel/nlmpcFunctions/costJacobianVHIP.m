function [G, Gmv, Ge] = costJacobianVHIP(X,U,~,data,modelProperties,stateLim,controlLim,stateWeight,controlWeight)
% generate analytical jacobians for the custom cost function



% model parameters extracted from passed parameters
z0 = modelProperties(3);
delta = stateLim(1);
zMax = stateLim(3);
zddotMax = controlLim(3);
xdotMax = stateLim(4);
zdotMax = stateLim(6);



% extract mpc details
p = data.PredictionHorizon;
Nx = data.NumOfStates;
Nmv = length(data.MVIndex);


G = zeros(p,Nx); 

G(1:p,1) = -2*stateWeight(1)*delta^(-2)*(data.References(1:p,1)-X(2:p+1,1));
G(1:p,2) = -2*stateWeight(3)*(zMax-z0)^(-2)*(data.References(1:p,2)-X(2:p+1,2));
G(1:p,3) = -2*stateWeight(4)*xdotMax^(-2)*(data.References(1:p,3)-X(2:p+1,3));
G(1:p,4) = -2*stateWeight(6)*zdotMax^(-2)*(data.References(1:p,4)-X(2:p+1,4));
Gmv = zeros(p,Nmv);

Gmv(1:p,1) = 2*controlWeight(1)*delta^(-2)*U(1:p,1);
Gmv(1:p,2) = 2*controlWeight(3)*zddotMax^(-2)*U(1:p,2);


Ge = 0;

end