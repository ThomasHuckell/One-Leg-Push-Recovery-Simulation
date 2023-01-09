function J = costFunctionVHIPPFW(X,U,~,data,modelProperties,stateLim,controlLim,stateWeight,controlWeight,controlRateWeight)
% This function is used to generate a custom cost function for MATLABs
% nonlinear MPC 

% model parameters extracted from passed parameters
z0 = modelProperties(3);
delta = stateLim(1);
thetaMax = stateLim(2);
zmax = stateLim(3);
tauMax = controlLim(2);
zddotMax = controlLim(3);
xdotMax = stateLim(4);
thetadotMax = stateLim(5);
zdotMax = stateLim(6);

% mpc prodiction horizon
p = data.PredictionHorizon;
Ts = data.Ts;

% weighting matrices for the cost function in the form J = x^TQx + u^TRu
Q = diag(stateWeight.*[delta^(-2),thetaMax^(-2),(zmax-z0)^(-2),xdotMax^(-2) ,thetadotMax^(-2),zdotMax^(-2)]);
R = diag(controlWeight.*[delta^(-2),tauMax^(-2),zddotMax^(-2)]);

Xerr = [zeros(1,size(data.References,2)); data.References] - X;

% initialize Cost
J = 0;


% add to the cost over prediction horizon
    for i = 2 : p+1
        
        J = J  + Ts*(Xerr(i,:)*Q*Xerr(i,:)' + U(i-1,:)*R*U(i-1,:)'  + controlRateWeight*(U(i,:)-U(i-1,:))*R*(U(i,:)-U(i-1,:))'  ) ; 
        
    end


end