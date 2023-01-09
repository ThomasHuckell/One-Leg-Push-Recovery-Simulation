function J = costFunctionLIPPFW(X,U,~,data,~,stateLim,controlLim,stateWeight,controlWeight)
% This function is used to generate a custom cost function for MATLABs
% nonlinear MPC 

% model parameters extracted from passed parameters

delta = stateLim(1);
thetaMax = stateLim(2);
tauMax = controlLim(2);
xdotMax = stateLim(4);
thetadotMax = stateLim(5);

% mpc prodiction horizon
p = data.PredictionHorizon;

% weighting matrices for the cost function in the form J = x^TQx + u^TRu
Q = diag(stateWeight([1 2 4 5]).*[delta^(-2),thetaMax^(-2),xdotMax^(-2),thetadotMax^(-2)]);
R = diag(controlWeight([1 2]).*[delta^(-2),tauMax^(-2)]);

Xerr = [zeros(1,size(data.References,2)); data.References] - X;

% initialize Cost
J = 0;


% add to the cost over prediction horizon
    for i = 2 : p+1
        
        J = J  + Xerr(i,:)*Q*Xerr(i,:)' + U(i-1,:)*R*U(i-1,:)' ; 
        
    end


end