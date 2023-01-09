function [X_com,X_knee,X_ank] = forwardKin(q,X_hip,phi,param)

% extract joint positions
q_hip  = q(1);
q_knee = q(2);


% extract model parmeters
legLength   = param(1);
torsoCoM    = param(2);
m           = param(3);

% determine position relative to hip
xB_torso = 0;
zB_torso = torsoCoM;


xB_thigh = (legLength)/2*cos(q_hip - pi/2);
zB_thigh = (legLength)/2*sin(q_hip - pi/2);

xB_knee = (legLength)*cos(q_hip - pi/2);
zB_knee = (legLength)*sin(q_hip - pi/2);


xB_shank = xB_knee + legLength/2*cos(q_hip + q_knee - pi/2);
zB_shank = zB_knee + legLength/2*sin(q_hip + q_knee - pi/2);

xB_ank = xB_knee + legLength*cos(q_hip + q_knee - pi/2);
zB_ank = zB_knee + legLength*sin(q_hip + q_knee - pi/2);

% homogenous transformation matrix of hip relative to world
T = [cos(phi), -sin(phi), X_hip(1);
     sin(phi), cos(phi), X_hip(2);
     0,0,1];
 
% determine CoM position

p_torso = 0.701339;
p_thigh = 0.205972;
p_shank = 0.092688;

XB_com = p_torso*[xB_torso;zB_torso] + p_thigh*[xB_thigh;zB_thigh] + p_shank*[xB_shank;zB_shank];


% joint Position
X_com  = T*[XB_com;1];
X_knee = T*[xB_knee;zB_knee;1];
X_ank  = T*[xB_ank;zB_ank;1];

X_com  = X_com(1:2,1);
X_knee = X_knee(1:2,1);
X_ank  = X_ank(1:2,1);



end