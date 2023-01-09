
q = zeros(1,3); %[0.5892   -1.0800    0.4908];

l_leg = 0.42;
l_torso = 0.3;

m_torso = 0.681;
m_thigh = 0.2;
m_shank = 0.09;

p_torso = 0.701339;
p_thigh = 0.205972;
p_shank = 0.092688;


hip = [0;0.94*0.84];
ankle = hip + l_leg*[cos(q(1) - pi/2);sin(q(1)-pi/2)] + l_leg*[cos(q(1) + q(2) - pi/2);sin(q(1) + q(2) - pi/2)];


CoM_hip = [0;0.25*l_torso]*p_torso + l_leg/2*[cos(q(1) - pi/2);sin(q(1)-pi/2)]*p_thigh + (l_leg*[cos(q(1) - pi/2);sin(q(1)-pi/2)] + ...
           l_leg/2*[cos(q(1) + q(2) - pi/2);sin(q(1) + q(2) - pi/2)])*p_shank 
       
       
       
       centerOfMass(oneLegRobotV2,[0,0,0])+[0,0,0.3]

       
CoM_ankle = l_leg/2*[cos(q(3) + pi/2);sin(q(3) + pi/2)]*p_shank + (l_leg*[cos(q(3) + pi/2) ;sin(q(3) + pi/2)] + ...
             l_leg/2*[cos(q(3) + q(2) + pi/2); sin(q(3) + q(2) + pi/2)] )*p_thigh + ...
             (l_leg*[cos(q(3) + pi/2) ;sin(q(3) + pi/2)] + l_leg*[cos(q(3) + q(2) + pi/2); sin(q(3) + q(2) + pi/2)] + 0.25*l_torso*[cos(sum(q)+pi/2); sin(sum(q) + pi/2)]  )*p_torso 


         centerOfMass(oneLegRobot,[0,0,0])
         

% figure()
% plot(out.rawzddot.Time,out.rawzddot.Data);
% 
% w = fft(out.rawzddot.Data(:,1));
% 
% L = length(out.rawzddot.Data(:,1));
% 
% Fs = f_sample;
% T = 1/Fs;
% 
% 
% f = Fs*(0:(L/2))/L;
% 
% P2 = abs(w/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% 
% figure()
% plot(f,P1);
% 
% 
% 
% [num,den] = butter(2,8/(Fs/2));
% 
% 
% lpFilt = designfilt('lowpassfir','PassbandFrequency',3/(Fs/2), ...
%          'StopbandFrequency',15/(Fs/2),'PassbandRipple',0.9, ...
%          'StopbandAttenuation',30,'DesignMethod','kaiserwin');
% 
% 
% 
% 
% filtData = filter(num,den,out.rawzddot.Data(:,1));
% 
% figure()
% plot(out.rawzddot.Time,filtData)
% hold on
% plot(out.rawzddot.Time,out.rawzddot.Data(:,1));
% 
% 
% pushArray = 50:5:125;
% for i = 1:16
%     
%     pushForce = pushArray(i);
%     
%     pushParam = struct('force',pushForce,'duration',pushDur,'onSet',pushOnset');
%     
%     
%     modelSim_LIP(i).pushParam = pushParam;
%     
%     
%     
% end
