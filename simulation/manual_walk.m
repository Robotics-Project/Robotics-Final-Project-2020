% manual_walk.m
% The main run file for seeing the model walk from a manual algorithm

% Simulation Parameters for model
simTime = 20;
simParameters = struct;
simParameters.StopTime = num2str(simTime); % units in seconds

walk


BRUpperLeg.signals.values=transition.BRUpper;
BLUpperLeg.signals.values=transition.BLUpper;

FRUpperLeg.signals.values=transition.FRUpper;
FLUpperLeg.signals.values=transition.FLUpper;

BRLowerLeg.signals.values=transition.BRLower;
BLLowerLeg.signals.values=transition.BLLower;

FRLowerLeg.signals.values=transition.FRLower;
FLLowerLeg.signals.values=transition.FLLower;

steps = numel(transition.BRUpper);

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


simOut = sim('robot.slx', simParameters);

% Axis-Angle Output and Distance Forward (X direction)
figure(1);
plot(simOut.axis.time, simOut.axis.data);
legend("X", "Y", "Z");
xlabel("Time (Seconds)");
title("axis of rotation");

figure(2);
plot(simOut.q.time, simOut.q.data);
ylabel("Angle (radians)");
xlabel("Time (Seconds)");
title("angle of rotation");

figure(3);
plot(simOut.distance_forward.time, simOut.distance_forward.data);
title("distance forward");
xlabel("Time (Seconds)");
ylabel("Distance (meters?)");