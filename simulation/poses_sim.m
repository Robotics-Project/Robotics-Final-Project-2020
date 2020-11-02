% poses_sim
% The interface file to hold the pose equations for the joints of the robot
% simulation

% Simulation Parameters for model
simParameters = struct;
simParameters.StopTime = '5';

% Angle Input Setup
% Back Right
BRUpperLeg.time=t;
BRUpperLeg.signals.dimensions=1;

% Back Left
BLUpperLeg.time=t;
BLUpperLeg.signals.dimensions=1;

% Front Right
FRUpperLeg.time=t;
FRUpperLeg.signals.dimensions=1;

% Front Left
FLUpperLeg.time=t;
FLUpperLeg.signals.dimensions=1;

% Back Right
BRLowerLeg.time=t;
BRLowerLeg.signals.dimensions=1;

% Back Left
BLLowerLeg.time=t;
BLLowerLeg.signals.dimensions=1;

% Front Right
FRLowerLeg.time=t;
FRLowerLeg.signals.dimensions=1;

% Front Left
FLLowerLeg.time=t;
FLLowerLeg.signals.dimensions=1;



% Setting Up Angle Data for Simulation
y = zeros(50,1);

% Half-Squat Constant
% bUpper = deg2rad(180-135);
% bLower = deg2rad(90+180);
% fUpper = deg2rad(180-225);
% fLower = deg2rad(270+180);

% Constant Arrays
% bUpper_constant = zeros(50,1);
% bUpper_constant(bUpper_constant == 0) = bUpper;
% 
% bLower_constant = zeros(50,1);
% bLower_constant(bLower_constant == 0) = bLower;
% 
% fUpper_constant = zeros(50,1);
% fUpper_constant(fUpper_constant == 0) = fUpper;
% 
% fLower_constant = zeros(50,1);
% fLower_constant(fLower_constant == 0) = fLower;

% Standing Constants
standing = struct;
standing.BRUpper = 0;
standing.BLUpper = 0;
standing.BRLower = 0;
standing.BLLower = 0;
standing.FRUpper = 0;
standing.FLUpper = 0;
standing.FRLower = 0;
standing.FLLower = 0;

% Sitting Constant
sitting = struct;
sitting.BRUpper = deg2rad(180-162.823146);
sitting.BLUpper = deg2rad(180-162.823146);
sitting.BRLower = deg2rad(59.610529+180);
sitting.BLLower = deg2rad(59.610529+180);
sitting.FRUpper = deg2rad(139.25-180);
sitting.FLUpper = deg2rad(139.25-180);
sitting.FRLower = deg2rad(0);
sitting.FLLower = deg2rad(0);

t = 0.1 * [0:49]';
transition = pose_transition(standing, sitting, t);


BRUpperLeg.signals.values=transition.BRUpper;
BLUpperLeg.signals.values=transition.BLUpper;

FRUpperLeg.signals.values=transition.FRUpper;
FLUpperLeg.signals.values=transition.FLUpper;

BRLowerLeg.signals.values=transition.BRLower;
BLLowerLeg.signals.values=transition.BLLower;

FRLowerLeg.signals.values=transition.FRLower;
FLLowerLeg.signals.values=transition.FLLower;


simOut = sim('robot.slx', simParameters);