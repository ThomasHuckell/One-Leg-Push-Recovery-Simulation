function robot = oneLegRobot(dataFormat)
%oneLegRobot Create rigidBodyTree for the robot model
%   ROBOT = oneLegRobot(DATAFORMAT) constructs a rigidBodyTree, ROBOT, and sets the
%   data format to DATAFORMAT. The possible values of DATAFORMAT are 
%   'struct', 'column' and 'row'. The default value is 'row', which 
%   matches the data format of the rigidbodytree object used to generate 
%   this function.

%   Auto-generated by MATLAB on 23-Feb-2022 23:47:41

%#codegen

narginchk(0,1);
if ~nargin==1
dataFormat='row';
end
robot = rigidBodyTree('MaxNumBodies', 4, 'DataFormat', dataFormat);
robot.Gravity = [                     0,                      0,                  -9.81];
robot.BaseName = 'base';


% Add body, 'foot', and joint, 'Ground Contact'
bodyName = 'foot';
bodyMass =                      0;
bodyCoM = [                     0,                      0,                      0];
bodyInertia = [                     0,                      0,                      0,                      0,                      0,                      0];
parentName = 'base';
jointName = 'Ground Contact';
jointType = 'fixed';
T_Joint_to_Parent = [                     0,                      1,                      0,                      0; ...
                                          0,                      0,                      1,                      0; ...
                                          1,                      0,                      0,                      0; ...
                                          0,                      0,                      0,                      1];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(T_Joint_to_Parent);
body = rigidBody(bodyName, "MaxNumCollisions", 0);

body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'shank', and joint, 'ankle Joint'
bodyName = 'shank';
bodyMass =                    6.3;
bodyCoM = [                 -0.21,                      0,                      0];
bodyInertia = [              0.124194,             0.00456315,               0.126132,                      0,                      0,                      0];
parentName = 'foot';
jointName = 'ankle Joint';
jointType = 'revolute';
dhparams = [                  0.42,                      0,                      0,                      0];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(dhparams, 'dh');
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [    -3.141592653589793,      3.141592653589793];
jointHomePosition =     0.4890104611477741;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;
body = rigidBody(bodyName, "MaxNumCollisions", 0);

body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'thigh', and joint, 'knee joint'
bodyName = 'thigh';
bodyMass =                     14;
bodyCoM = [                 -0.21,                      0,                      0];
bodyInertia = [               0.27727,              0.0114237,               0.280294,                      0,                      0,                      0];
parentName = 'shank';
jointName = 'knee joint';
jointType = 'revolute';
dhparams = [                  0.42,                      0,                      0,                      0];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(dhparams, 'dh');
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [    -3.141592653589793,      3.141592653589793];
jointHomePosition =     -1.079738863212149;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;
body = rigidBody(bodyName, "MaxNumCollisions", 0);

body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


% Add body, 'torso', and joint, 'hip joint'
bodyName = 'torso';
bodyMass =                  47.67;
bodyCoM = [                -0.235,                      0,                      0];
bodyInertia = [                 1.367,                  0.094,             5.36742425,                      0,                      0,                      0];
parentName = 'thigh';
jointName = 'hip joint';
jointType = 'revolute';
dhparams = [                  0.47,                      0,                      0,                      0];

joint = rigidBodyJoint(jointName, jointType);
joint.setFixedTransform(dhparams, 'dh');
jointAxis = [                     0,                      0,                      1];
jointPositionLimits = [    -3.141592653589793,      3.141592653589793];
jointHomePosition =     0.5907284020643746;
joint.PositionLimits = jointPositionLimits;
joint.JointAxis = jointAxis;
joint.HomePosition = jointHomePosition;
body = rigidBody(bodyName, "MaxNumCollisions", 0);

body.Joint = joint;
body.Mass = bodyMass;
body.CenterOfMass = bodyCoM;
body.Inertia = bodyInertia;
robot.addBody(body, parentName);


