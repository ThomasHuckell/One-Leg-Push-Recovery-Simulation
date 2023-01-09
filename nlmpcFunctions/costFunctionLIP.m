function J = costFunctionLIP(X,U,~,data,~,stateLim,~,stateWeight,controlWeight,controlRateWeight)
% This function is used to generate a custom cost function for MATLABs
% nonlinear MPC 

% model parameters extracted from passed parameters
delta = stateLim(1);
xdotMax = stateLim(4);


% mpc prodiction horizon
p = data.PredictionHorizon;
Ts = data.Ts;
% weighting matrices for the cost function in the form J = x^TQx + u^TRu
Q = diag(stateWeight([1 4]).*[delta^(-2),xdotMax^(-2)]);
R = controlWeight(1)*delta^(-2);

Xerr = [zeros(1,size(data.References,2)); data.References] - X;

% initialize Cost
J = 0;


% add to the cost over prediction horizon
    for i = 2 : p+1
        
        J = J + Ts*(Xerr(i,:)*Q*Xerr(i,:)' + U(i-1,:)*R*U(i-1,:)' + controlRateWeight*(U(i,:)-U(i-1,:))*R*(U(i,:)-U(i-1,:))' );
        
    end


end