%% Simscape multiBody model of leg prototype

%% model Parameters $$$ make into seperate script

robot = rigidBodyTree;


dhparams = [0 pi/2 0 pi;
            legLength 0 0 0;
            legLength 0 0 0;
            torsoLength 0 0 0];



body1 = rigidBody('foot');

jnt1 = rigidBodyJoint('Ground Contact','fixed');


body1.Mass = 0;
body1.CenterOfMass = [0,0,0];
body1.Inertia = [0 0 0 0 0 0];



body2 = rigidBody('shank');
jnt2 = rigidBodyJoint('ankle Joint','revolute');
body2.Mass = segmentMass.shank*m;
body2.CenterOfMass = [-legLength/2,0,0];
body2.Inertia = [0.124194, 0.00456315, 0.126132 0 0 0];
jnt2.HomePosition = t3_0;


body3 = rigidBody('thigh');
jnt3 = rigidBodyJoint('knee joint','revolute');
jnt3.HomePosition = t2_0;
body3.Mass = segmentMass.thigh*m;
body3.CenterOfMass = [-legLength/2,0,0]; 
body3.Inertia = [0.27727, 0.0114237, 0.280294 0 0 0];


body4 = rigidBody('torso');
jnt4 = rigidBodyJoint('hip joint','revolute');
jnt4.HomePosition = t1_0;
body4.Mass = segmentMass.torso*m;
body4.CenterOfMass = [-torsoCoM,0,0];
body4.Inertia = [1.367 0.094 (8 - segmentMass.torso*m*(torsoCoM)^2 ) 0 0 0];


setFixedTransform(jnt1,[0,1,0,0;0,0,1,0;1,0,0,0;0,0,0,1]);
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');


body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;


addBody(robot,body1,'base')
addBody(robot,body2,'foot')
addBody(robot,body3,'shank')
addBody(robot,body4,'thigh')

robot.Gravity = [0 0 -9.81];

show(robot)

robot.DataFormat = 'row';

writeAsFunction(robot,'oneLegRobot')

