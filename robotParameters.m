%% Physical parameters

% Support Surface specifications [m]
support_x = 2.5;  
support_y = 2.5;
support_z = 0.1;

% foot size
delta = 0.3;           % total foot length [m]
deltaPlus  = 0.20; % distance of ankle to toe [m]

% extrude solid dimensions
foot_x = delta;         
foot_y = delta/3;
foot_z = 0.03;


% limbs dimensions
legWidth  = 0.08;       % leg limb width [m]
legLength = 0.42;        % leg limb length [m]

torsoWidth  = 0.14;      % torso limb width [m]
torsoLength = 0.47;       % torso limb length [m]
torsoCoM    = 2/4*torsoLength; % CoM location from the top of the torso Limb

connectorRad  = 0.01;    
limbThickness = 0.05;  % limb thickness [m]

ankleOffset = legWidth/2+foot_z;    % height of ankle joint above contact surface [m]

% generate extrude geometry coordnates 
torso_xy = genLimbCoord(torsoWidth,torsoLength,0); 
thigh_xy = genLimbCoord(legWidth,legLength,0);
shank_xy = genLimbCoord(legWidth,legLength,0);
ank_xy = genAnkCoord(connectorRad*2,deltaPlus,delta,0.75);



%% Reduced Order Model parameters
m = 70;                     % mass
J = 8;                      % inertia
tauMax    = 100;            % maximum hip torque
zddotMax  = 3;              % maximum vertical Acceleration
thetaMax  = 0.5;            % maximum flywheel position
zmin      = 0.6+ ankleOffset;            % minimum vertical position
zmax      = (2*legLength)*sin(acos(0.2/(2*legLength)))+ankleOffset ;   % maximum vertical postion



% segment mass distribution
segmentMass = struct('torso',0.681, 'thigh', 0.2, 'shank', 0.09, 'foot', 0.029);    % segment mass distribution


% initial pose
L = 0.94*2*legLength;               % initial vertical leg length [m] (relative to ankle)

%z0 = L + ankleOffset;               % rest height [m]

parameters = struct('mass',m,'inertia',J,'gravity',9.81,'restHeight',L ,'supportSize',[-(delta-deltaPlus) deltaPlus],...
                    'tauMax',tauMax,'thetaMax',thetaMax,'zddot',zddotMax,'zMax',(2*legLength)*sind(75.9) + ankleOffset);


[t1_0,t2_0,t3_0] = initPose(legLength,legLength,ankleOffset, L );

robot = oneLegRobot;

Tau_0 = inverseDynamics(robot);
CoM_0 = centerOfMass(robot);

% initial contact forces
f_0 = [1,1; deltaPlus-CoM_0(1), -(delta-deltaPlus+CoM_0(1))]\[m*9.81/2;0];


%% Controller parameters
% Ankle PID gains
kp_ank = 375 ;          %325; 400 VHIP
kd_ank = 0;
ki_ank = 1800;           %500; 100 VHIP

% Hip PID gains
kp_hip = 500;%10;
kd_hip = 250;%5;
ki_hip = 0;%0.3;

% Knee PID gains zzdot tracking
kp_knee1 = 8;
kd_knee1 = 0.02;
ki_knee1 = 0;

% Knee PID gain constant height tracking
kp_knee2 = 1000; %1000 %400;
kd_knee2 = 500; %500 %200;
ki_knee2 = 0; %0;

filterCoef = 50;

%% Ground Contact parameters

k_contact = 4e5;    %4e5 support Surface Stiffness [N/m]
c_contact = 4e4;    %4e4 support Surface Damping   [Ns/m]

mu_s = 0.9;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 0.8;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.025;   % Friction velocity threshold (m/s)


%% Sensor Filter parameters

f_sample = 500;

f_CoP = 17.5;

f_zddot = 12; %3

f_qdot  = 50;

f_tau = 100;




[num_qdot, den_qdot] = butter(2,f_qdot/(f_sample/2));

[num_zddot, den_zddot] = butter(2,f_zddot/(f_sample/2));

[num_CoP, den_CoP]  = butter(3,f_CoP/(f_sample/2));

[num_tau, den_tau] = butter(4,f_tau/(f_sample/2));

    

%% function to determine initial joint angles

function [t1,t2,t3] = initPose(l1,l2,offset,z0)
    


syms f(theta1,theta2)
    
p_torso = 0.701339;
p_thigh = 0.205972;
p_shank = 0.092688;

    f = [l1*cos(theta1)+ l1*cos(theta2) + offset;
         l1*sin(theta1)+ l1*sin(theta2)] == [z0;(l1/2*sin(theta1))*p_thigh + (l1*sin(theta1)+l2/2*sin(theta2))*p_shank ];%  

    sol = solve(f,'Real',true);
 
t1 = eval(sol.theta1(1)) ;
t2 = eval(sol.theta2(1)) - t1;

t3 = -t1 -t2;


end
%% function for generating extruded solids geometry 
function leg_xy = genLimbCoord(legWidth,legLength,connectorRad)
    
    theta = linspace(2*pi,0);
    
    N = length(theta);
    
    c = 1;
    
    if (connectorRad == 0)
        
        leg_xy = zeros(200,2);
        
        
    else
        
        leg_xy = zeros(301,2);
        
        leg_xy(201,:) = [legWidth/2,0];
        
        for i = 1:N
        
            leg_xy(201+i,:) = connectorRad*[cos(theta(i)),sin(theta(i))];
        
        end
        
    end
    
    theta = linspace(0,pi);
    
    for i = 1:N
        
        leg_xy(c,:) = legWidth/2*[cos(theta(i)),sin(theta(i))];
        
        c =  c + 1;
        
    end
    
    theta = linspace(pi,2*pi);
    
    for i = 1:N
        
        leg_xy(c,:) = legWidth/2*[cos(theta(i)),sin(theta(i))] + [0,-legLength];
        
        c =  c + 1;
        
    end

end


function ank_xy = genAnkCoord(h,deltaPlus,delta,alpha)
    

    ank_xy = [0,0;
              alpha*deltaPlus,0;
              0,h;
              -alpha*(delta-deltaPlus),0];
   
    
end
