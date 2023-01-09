%% Run Prior to Simulink model
clear 
clc
close all

addpath('reducedOrderSim','reducedOrderSim','plottingFunctions','MultiBodyModel','animation','nlmpcFunctions','helperFunctions')


% set which reduced order model to use
% VHIPPFW: Variable Height Inverted Pendulum Plus Flywheel      model = 4;
% VHIP: Variable Height Inverted Pendulum                       model = 3;
% LIPPFW: Linear Inverted Pendulum Plus Flywheel                model = 2;
% LIP: Linear Inverted Pendulum                                 model = 1;
model = 2;

% load simulink model parameters
robotParameters


% load nlmpc parameters
initNLMPC

% set base disturbance values
simTime = 5;
pushDur = 0.1;
pushOnset = 0.2;
pushForce = 494;            

pushParam = struct('force',pushForce,'duration',pushDur,'onSet',pushOnset);

% set to 1 if you want to manually test simulink model
manualSimFlag = 1;

animationFlag = 0;

% set to 1 if you want to simulate a range push recovery across the 4 reduced order
% models 
pushDataGenFlag = 0;

%%
if manualSimFlag == 1
    set_param(mdl,'SimMechanicsOpenEditorOnUpdate','on');
    
    simParam.nlParameters.controlLim = controlLim;
    simParam.nlParameters.stateLim = stateLim;
    simParam.nlParameters.modelProperties = modelProperties;
    simParam.nlParameters.stateWeight = stateWeight;
    simParam.nlParameters.controlWeight = controlWeight;
    simParam.nlParameters.controlRateWeight = controlRateWeight;
    simParam.model = model;
    
    
    switch model
        case 1
            simParam.nlobj = nlobjLIP;
        case 2
            simParam.nlobj = nlobjLIPPFW;
        case 3
            simParam.nlobj = nlobjVHIP;
        case 4
            simParam.nlobj = nlobjVHIPPFW; 
    end
           

    
    tic
    modelSim = reducedOrderSim(pushParam,simParam,simTime);
    t = toc
    
    
    plotSim(modelSim,model,0);
    
end


if pushDataGenFlag == 1
    
    genPushData();
    
end


%%
if animationFlag == 1
    IP_animation(modelSim,simParam,1)
end

