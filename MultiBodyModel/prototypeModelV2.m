%% Simscape multiBody model of leg prototype

%% model Parameters $$$ make into seperate script

robot = rigidBodyTree;


dhparams = [torsoLength pi/2 0 -pi/2;
            legLength 0 0 0;
            legLength 0 0 0;
            0 0 0 0];



body1 = rigidBody('torso');

jnt1 = rigidBodyJoint('BodyFrame','fixed');




body1.Mass = segmentMass.torso*m;
body1.CenterOfMass = [-0.25*torsoLength,0,0];
body1.Inertia = [0 0 J 0 0 0];








body2 = rigidBody('thigh');
jnt2 = rigidBodyJoint('knee joint','revolute');
jnt2.HomePosition = t1_0;
body2.Mass = segmentMass.thigh*m;
body2.CenterOfMass = [-legLength/2,0,0]; 
body2.Inertia = [0.253921, 0.0101298, 0.258218 0 0 0];

body3 = rigidBody('shank');
jnt3 = rigidBodyJoint('ankle Joint','revolute');
body3.Mass = segmentMass.shank*m;
body3.CenterOfMass = [-legLength/2,0,0];
body3.Inertia = [0.114264, 0.00455842, 0.116198 0 0 0];
jnt3.HomePosition = t2_0;

body4 = rigidBody('ankle');
jnt4 = rigidBodyJoint('hip joint','revolute');
jnt4.HomePosition = t3_0;
body4.Mass = 0;
body4.CenterOfMass = [0,0,0];
body4.Inertia = [0 0 0 0 0 0];

setFixedTransform(jnt1,[0,1,0,0;0,0,-1,0;-1,0,0,-torsoLength;0,0,0,1]);
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;

addBody(robot,body1,'base')
addBody(robot,body2,'torso')
addBody(robot,body3,'thigh')
addBody(robot,body4,'shank')

robot.Gravity = [0 0 -9.81];

show(robot)

robot.DataFormat = 'row';

writeAsFunction(robot,'oneLegRobotV2')

