% This script initilizes the nlmpc controller

% set path to the folder of the nlmpc Functions

mdl = 'robotPrototype';

open(mdl)

% Turn off fast restart, cannot create nlmpc bus without it off
set_param(mdl,'FastRestart','off')

% Create parameter variables for MPC
modelProperties = [m, J, L];
stateLim     = [deltaPlus,thetaMax,zmax,0.593, tauMax/J*sqrt(J*thetaMax/tauMax),zddotMax*sqrt((zmax-(L))/zddotMax)]; 
controlLim   = [deltaPlus,tauMax,zddotMax];
stateWeight  = [3.5 0.9 0.6 12 0.81 0.36];  % %[3 0.5 0.3 9 0.25 0.09];
controlWeight = [1 0.9 0.5];  % [x_c tau zddot] %[0.75 0.6 0.5]
controlRateWeight = 2.5;


% constant nlmpc parameters
Ts = 0.05;      % sampling time
p  = 14;        % predition horizon
c  = 2;         % control horizon


%% VHIPPFW NLMPC
nx = 6;     % # states
ny = 6;     % # outputs
nu = 3;     % # manipulated varaibles

% create non linear mpc object with 6 states, 6 outputs and 3 manipulated
% variables
nlobjVHIPPFW = nlmpc(nx,ny,nu);


nlobjVHIPPFW.Ts = Ts;                  % set sample time
nlobjVHIPPFW.PredictionHorizon = p;    % set prediction horizon
nlobjVHIPPFW.ControlHorizon = c;       % set control horizon


% Set state and jacobian functions of the model
nlobjVHIPPFW.Model.StateFcn = "stateVHIPPFW";
nlobjVHIPPFW.Jacobian.StateFcn = "jacVHIPPFW";
nlobjVHIPPFW.Optimization.CustomCostFcn = "costFunctionVHIPPFW";
nlobjVHIPPFW.Jacobian.CustomCostFcn = "costJacobianVHIPPFW";
nlobjVHIPPFW.Model.NumberOfParameters = 6;


% Set limits on manipulated variable
nlobjVHIPPFW.MV = struct('Min',{-(delta-deltaPlus),-tauMax,-zddotMax},'Max',{deltaPlus,tauMax,zddotMax});
nlobjVHIPPFW.OutputVariables = struct('Min',{-1.5*(delta-deltaPlus),-thetaMax,zmin,-10,-30,-9.81},'Max',{1.5*deltaPlus,thetaMax,zmax,10,30,9.81});

% Set weights for NLMPC with priority on output commented if using custom
% cost function
% nlobjVHIPPFW.Weights.OutputVariables =             [3 0.5 0.5 6 0 0];
% nlobjVHIPPFW.Weights.ManipulatedVariables =        [0.1 0.1 0.1];
% nlobjVHIPPFW.Weights.ManipulatedVariablesRate =    [1 1 1];



createParameterBus(nlobjVHIPPFW,[mdl '/Controller/MPC/VHIP/Nonlinear MPC Controller'],'myBusObjectVHIPPFW',{modelProperties,stateLim,controlLim,stateWeight,controlWeight,controlRateWeight})

%% VHIP NLMPC

nx = 4;     % # states
ny = 4;     % # outputs
nu = 2;     % # manipulated varaibles

% create non linear mpc object with 6 states, 6 outputs and 3 manipulated
% variables
nlobjVHIP = nlmpc(nx,ny,nu);



nlobjVHIP.Ts = Ts;                 % set sample time
nlobjVHIP.PredictionHorizon = p;   % set prediction horizon
nlobjVHIP.ControlHorizon = c;       % set control horizon


% Set state and jacobian functions of the model
nlobjVHIP.Model.StateFcn = "stateVHIP";
nlobjVHIP.Jacobian.StateFcn = "jacVHIP";
nlobjVHIP.Optimization.CustomCostFcn = "costFunctionVHIP";
nlobjVHIP.Jacobian.CustomCostFcn = "costJacobianVHIP";
nlobjVHIP.Model.NumberOfParameters = 6;


% Set limits on manipulated variable
nlobjVHIP.MV = struct('Min',{-(delta-deltaPlus),-zddotMax},'Max',{deltaPlus,zddotMax});
nlobjVHIP.OutputVariables = struct('Min',{-1.5*(delta-deltaPlus),zmin,-10,-9.81},'Max',{1.5*deltaPlus,zmax,10,9.81});

% Set weights for NLMPC with priority on output commented if using custom
% cost function
% nlobjVHIP.Weights.OutputVariables =             [3 3 0 0];
% nlobjVHIP.Weights.ManipulatedVariables =        [0.1 0.1 ];
% nlobjVHIP.Weights.ManipulatedVariablesRate =    [1 1];

createParameterBus(nlobjVHIP,[mdl '/Controller/MPC/VHIP/Nonlinear MPC Controller'],'myBusObjectVHIP',{modelProperties,stateLim,controlLim,stateWeight,controlWeight,controlRateWeight})

%% LIPPFW NLMPC

nx = 4;     % # states
ny = 4;     % # outputs
nu = 2;     % # manipulated varaibles

% create non linear mpc object with 6 states, 6 outputs and 3 manipulated
% variables
nlobjLIPPFW = nlmpc(nx,ny,nu);


nlobjLIPPFW.Ts = Ts;                 % set sample time
nlobjLIPPFW.PredictionHorizon = p;   % set prediction horizon
nlobjLIPPFW.ControlHorizon = c;       % set control horizon


% Set state and jacobian functions of the model
nlobjLIPPFW.Model.StateFcn = "stateLIPPFW";
nlobjLIPPFW.Jacobian.StateFcn = "jacLIPPFW";
nlobjLIPPFW.Optimization.CustomCostFcn = "costFunctionLIPPFW";
nlobjLIPPFW.Jacobian.CustomCostFcn = "costJacobianLIPPFW";
nlobjLIPPFW.Model.NumberOfParameters = 6;


% Set limits on manipulated variable
nlobjLIPPFW.MV = struct('Min',{-(delta-deltaPlus),-tauMax},'Max',{deltaPlus,tauMax});
nlobjLIPPFW.OutputVariables = struct('Min',{-1.5*(delta-deltaPlus),-thetaMax,-10,-30},'Max',{1.5*deltaPlus,thetaMax,10,30});

% Set weights for NLMPC with priority on output commented if using custom
% cost function
%  nlobjLIPPFW.Weights.OutputVariables =             stateWeight([1,2,4,5]).*stateLim([1,2,4,5]);
%  nlobjLIPPFW.Weights.ManipulatedVariables =        controlWeight([1,2]).*controlLim([1,2]);
%  nlobjLIPPFW.Weights.ManipulatedVariablesRate =    [0 0];

createParameterBus(nlobjLIPPFW,[mdl '/Controller/MPC/LIPPFW/Nonlinear MPC Controller'],'myBusObjectLIPPFW',{modelProperties,stateLim,controlLim,stateWeight,controlWeight,controlRateWeight})

%% LIP NLMPC

nx = 2;     % # states
ny = 2;     % # outputs
nu = 1;     % # manipulated varaibles



% create non linear mpc object with 6 states, 6 outputs and 3 manipulated
% variables
nlobjLIP = nlmpc(nx,ny,nu);


nlobjLIP.Ts = Ts;                 % set sample time
nlobjLIP.PredictionHorizon = p;   % set prediction horizon
nlobjLIP.ControlHorizon = c;       % set control horizon


% Set state and jacobian functions of the model
nlobjLIP.Model.StateFcn = "stateLIP";
nlobjLIP.Jacobian.StateFcn = "jacLIP";
nlobjLIP.Optimization.CustomCostFcn = "costFunctionLIP";
nlobjLIP.Jacobian.CustomCostFcn = "costJacobianLIP";
nlobjLIP.Model.NumberOfParameters = 6;


% Set limits on manipulated variable
nlobjLIP.MV = struct('Min',{-(delta-deltaPlus)},'Max',{deltaPlus});
nlobjLIP.OutputVariables = struct('Min',{-1.5*(delta-deltaPlus),-10},'Max',{1.5*deltaPlus,10});

% Set weights for NLMPC with priority on output commented if using custom
% cost function
% nlobjLIP.Weights.OutputVariables =             [3 0];
% nlobjLIP.Weights.ManipulatedVariables =        0.1;
% nlobjLIP.Weights.ManipulatedVariablesRate =    1;

createParameterBus(nlobjLIP,[mdl '/Controller/MPC/LIP/Nonlinear MPC Controller'],'myBusObjectLIP',{modelProperties,stateLim,controlLim,stateWeight,controlWeight,controlRateWeight})