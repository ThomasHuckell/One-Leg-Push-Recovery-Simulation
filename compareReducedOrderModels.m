% load simulink model parameters
addpath('reducedOrderSim','nlmpcFunctions','reducedOrderSim','plottingFunctions','MultiBodyModel','data')


robotParameters

robot = oneLegRobot;
% load nlmpc parameters
initNLMPC

% set base disturbance values
simTime = 5;
pushDur = 0.1;
pushOnset = 0.2;

pushParam =  struct('force',0,'duration',pushDur,'onSet',pushOnset);

simParam.nlParameters.controlLim = controlLim;
simParam.nlParameters.stateLim = stateLim;
simParam.nlParameters.modelProperties = modelProperties;
simParam.nlParameters.stateWeight = stateWeight;
simParam.nlParameters.controlWeight = controlWeight;
simParam.nlParameters.controlRateWeight = controlRateWeight;


simParam.model = 1;
simParam.nlobj = nlobjLIP;



pushStart = 20;
pushStep  = 20;


modelSim_LIP = genModelData(simParam,pushParam,simTime,pushStart,pushStep); 


simParam.model = 2;
simParam.nlobj = nlobjLIPPFW;
 

modelSim_LIPPFW = genModelData(simParam,pushParam,simTime,pushStart,pushStep); 


simParam.model = 3;
simParam.nlobj = nlobjVHIP; 

modelSim_VHIP = genModelData(simParam,pushParam,simTime,pushStart,pushStep); 
   
    simParam.model = 4;
    simParam.nlobj = nlobjVHIPPFW;

modelSim_VHIPPFW = genModelData(simParam,pushParam,simTime,pushStart,pushStep); 


 
% save data
fileName = strcat('data/modelPushData','_Dur',num2str(pushDur,'%.0e'),'_',datestr(now,'mm-dd-yyyy HH-MM'),'.mat');

costFcn = struct('stateWeights',stateWeight,'controlWeights',controlWeight,'controlRateWeight',controlRateWeight);
nlmpcInfo   = struct('LIP',nlobjLIP,'LIPPFW',nlobjLIPPFW,'VHIP',nlobjVHIP,'VHIPPFW',nlobjVHIPPFW);
simData = struct('LIP',modelSim_LIP,'LIPPFW',modelSim_LIPPFW,'VHIP',modelSim_VHIP,'VHIPPFW',modelSim_VHIPPFW);
pushSim = struct('type','Reduced Order Model','parameters',parameters,'nlmpcInfo',nlmpcInfo,'costFcnInfo',costFcn,'simData',simData);


save(fileName,'pushSim')


function modelData = genModelData(simParam,pushParam,simTime,pushStart,pushStep)

pushFail  = inf;
pushSuccess = 0;
pushForce = pushStart;

c = 1;


while(true) % run until break
    
    pushParam.force = pushForce;

    sim = reducedOrderSim(pushParam,simParam,simTime);
    
    if sim.time(end) == simTime     % simulation completed
        
        pushSuccess = pushForce;
        
        modelData(c) = sim;
        pushForce = pushForce + pushStep;       % increase push and advance
          
        c = c + 1;
        
    else
        pushFail = pushForce;                % Update smallest failure push force
        
        pushStep = max(ceil(pushStep/4),1); % reduce pushStep by quarter or to one
        
        
        pushForce = pushForce - 2*pushStep; % Reduce push to half step size
        
        
        
    end
    
    
    if (pushFail-pushSuccess) <= 1
        break
    end

end

end
