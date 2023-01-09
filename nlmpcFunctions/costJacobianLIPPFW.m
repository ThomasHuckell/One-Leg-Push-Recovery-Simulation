function [G, Gmv, Ge] = costJacobianLIPPFW(X,U,~,data,~,stateLim,controlLim,stateWeight,controlWeight,controlRateWeight)
% generate analytical jacobians for the custom cost function

% model parameters extracted from passed parameters
delta = stateLim(1);
thetaMax = stateLim(2);
tauMax = controlLim(2);
xdotMax = stateLim(4);
thetadotMax = stateLim(5);


% extract mpc details
p = data.PredictionHorizon;
Nx = data.NumOfStates;
Nmv = length(data.MVIndex);
Ts  = data.Ts;

G = zeros(p,Nx); 

G(1:p,1) = Ts*(-2*stateWeight(1)*delta^(-2)*(data.References(1:p,1)-X(2:p+1,1)));
G(1:p,2) = Ts*(-2*stateWeight(2)*thetaMax^(-2)*(data.References(1:p,2)-X(2:p+1,2)));
G(1:p,3) = Ts*(-2*stateWeight(4)*xdotMax^(-2)*(data.References(1:p,3)-X(2:p+1,3)));
G(1:p,4) = Ts*(-2*stateWeight(5)*thetadotMax^(-2)*(data.References(1:p,4)-X(2:p+1,4)));



Gmv = zeros(p,Nmv);

Gmv(1,1) = Ts*(2*controlWeight(1)*delta^(-2)*U(1,1) - 2*controlRateWeight*controlWeight(1)*delta^(-2)*(U(2,1) - U(1,1)) );
Gmv(2:p,1) = Ts*(2*controlWeight(1)*delta^(-2)*U(2:p,1) + 2*controlRateWeight*controlWeight(1)*delta^(-2)*( -U(3:p+1,1) + 2*U(2:p,1) - (U(1:p-1,1)) ) );

Gmv(1,2) = Ts*(2*controlWeight(2)* tauMax^(-2)*U(1,2) - 2*controlRateWeight*controlWeight(2)*tauMax^(-2)*(U(2,2) - U(1,2)) );
Gmv(2:p,2) = Ts*(2*controlWeight(2)*tauMax^(-2)*U(2:p,2) + 2*controlRateWeight*controlWeight(2)*tauMax^(-2)*( -U(3:p+1,2) + 2*U(2:p,2) - (U(1:p-1,2)) ) );



Ge = 0;

end