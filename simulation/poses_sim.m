% poses_sim
% The interface file to hold the pose equations for the joints of the robot
% simulation

% Running Simulation from model
simParameters = struct;
simParameters.StopTime = '10';

% Setting Up Angle Data for Simulation
t = 0.2 * [0:49]';
x = cos(t);
y = zeros(50,1);
z = -cos(t);
BRUpperLeg.time=t;
BRUpperLeg.signals.values=y;
BRUpperLeg.signals.dimensions=1;

BLUpperLeg.time=t;
BLUpperLeg.signals.values=y;
BLUpperLeg.signals.dimensions=1;

FRUpperLeg.time=t;
FRUpperLeg.signals.values=y;
FRUpperLeg.signals.dimensions=1;

FLUpperLeg.time=t;
FLUpperLeg.signals.values=y;
FLUpperLeg.signals.dimensions=1;

BRLowerLeg.time=t;
BRLowerLeg.signals.values=y;
BRLowerLeg.signals.dimensions=1;

BLLowerLeg.time=t;
BLLowerLeg.signals.values=y;
BLLowerLeg.signals.dimensions=1;

FRLowerLeg.time=t;
FRLowerLeg.signals.values=y;
FRLowerLeg.signals.dimensions=1;

FLLowerLeg.time=t;
FLLowerLeg.signals.values=y;
FLLowerLeg.signals.dimensions=1;



simOut = sim('robot.slx', simParameters);

% 8 Joint Equations over t = 0: