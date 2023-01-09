function J = objCost(data,modelParam,costParam,model)

%% extract data
J = modelParam.inertia;
delta = modelParam.supportSize;
z0 = modelParam.restHeight;


xMax = [modelParam.supportSize(2),modelParam.thetaMax,modelParam.zMax,0.593,...
    modelParam.tauMax/J*sqrt(J*modelParam.thetaMax/modelParam.tauMax),...
    modelParam.zddot*sqrt(modelParam.zMax-z0)/modelParam.zddot];
uMax = [delta(2),modelParam.tauMax,modelParam.zddot];


sampleTime = 0.05:0.05:5;

ind = zeros(length(sampleTime),1);


for i = 1:length(sampleTime)
    [~,temp] = min(abs(sampleTime(i)-data.time));
    ind(i) = temp;
end

X = data.state(ind,:);
U = data.control(ind,:);


%%

switch model
    case "LIP"
        Q = diag( xMax([1,4]).^-2.*costParam.stateWeights([1,4]) );
        R = diag(uMax(1).^-2.*costParam.controlWeights(1));
        
        X0 = zeros(1,2);
    case "LIPPFW"
        Q = diag( xMax([1,2,4,5]).^-2.*costParam.stateWeights([1,2,4,5]) );
        R = diag(uMax(1,2).^-2.*costParam.controlWeights(1,2));
        
        X0 = zeros(1,4);
    case "VHIP"
        Q = diag( xMax([1,3,4,6]).^-2.*costParam.stateWeights([1,3,4,6]) );
        R = diag(uMax([1,3]).^-2.*costParam.controlWeights([1,3]));
        
        X0 = [0 z0 0 0];
       
    case "VHIPPFW"
        
        Q = diag( xMax.^-2.*costParam.stateWeights);
        R = diag( uMax.^-2.*costParam.controlWeights);
        
        X0 = [0 0 z0 0 0 0];
end


J = 0;

for i = 1:length(sampleTime)-1
    
    J = J + 0.05*((X(i,:)-X0)*Q*(X(i,:)-X0)' + U(i,:)*R*U(i,:)' + costParam.controlRateWeight*((U(i+1,:)-U(i,:))*R*(U(i+1,:)-U(i,:))' ) );
    
end



end