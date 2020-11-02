% poses_sim
% The interface file to hold the pose equations for the joints of the robot
% simulation

% Running Simulation from model
simParameters = struct;
simParameters.StopTime = '10';

% Setting Up Angle Data for Simulation
t = 0.2 * [0:49]';
y = zeros(50,1);

% Standing Constant
% b1 = 0;
% b2 = 0;
% f1 = 0;
% f2 = 0;

% Half-Squat Constant
% b1 = deg2rad(180-135);
% b2 = deg2rad(90+180);
% f1 = deg2rad(180-225);
% f2 = deg2rad(270+180);

% Sitting Constant
b1 = deg2rad(180-162.823146);
b2 = deg2rad(59.610529+180);
f1 = deg2rad(139.25-180);
f2 = deg2rad(0);

b1_constant = zeros(50,1);
b1_constant(b1_constant == 0) = b1;

b2_constant = zeros(50,1);
b2_constant(b2_constant == 0) = b2;

f1_constant = zeros(50,1);
f1_constant(f1_constant == 0) = f1;

f2_constant = zeros(50,1);
f2_constant(f2_constant == 0) = f2;

% Back Right
BRUpperLeg.time=t;
BRUpperLeg.signals.values=b1_constant;
BRUpperLeg.signals.dimensions=1;

% Back Left
BLUpperLeg.time=t;
BLUpperLeg.signals.values=b1_constant;
BLUpperLeg.signals.dimensions=1;

% Front Right
FRUpperLeg.time=t;
FRUpperLeg.signals.values=f1_constant;
FRUpperLeg.signals.dimensions=1;

% Front Left
FLUpperLeg.time=t;
FLUpperLeg.signals.values=f1_constant;
FLUpperLeg.signals.dimensions=1;

% Back Right
BRLowerLeg.time=t;
BRLowerLeg.signals.values=b2_constant;
BRLowerLeg.signals.dimensions=1;

% Back Left
BLLowerLeg.time=t;
BLLowerLeg.signals.values=b2_constant;
BLLowerLeg.signals.dimensions=1;

% Front Right
FRLowerLeg.time=t;
FRLowerLeg.signals.values=f2_constant;
FRLowerLeg.signals.dimensions=1;

% Front Left
FLLowerLeg.time=t;
FLLowerLeg.signals.values=f2_constant;
FLLowerLeg.signals.dimensions=1;



simOut = sim('robot.slx', simParameters);

% 8 Joint Equations over t = 0: