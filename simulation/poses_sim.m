% poses_sim
% The interface file to hold the pose equations for the joints of the robot
% simulation

% Simulation Parameters for model

simTime = 20;
simParameters = struct;
simParameters.StopTime = num2str(simTime); % units in seconds

% Pose Angles for each pose
% Half-Squat Constant
squat = struct;
squat.BRUpper = deg2rad(180-135);
squat.BLUpper = deg2rad(180-135);
squat.BRLower = deg2rad(90+180);
squat.BLLower = deg2rad(90+180);
squat.FRUpper = deg2rad(180-225);
squat.FLUpper = deg2rad(180-225);
squat.FRLower = deg2rad(270+180);
squat.FLLower = deg2rad(270+180);

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


% Generate a List of Commands
% commands = [0, 1, 2, 0, 2, 1, 0, 0, 1];
actionListNum = [2;2;0;2;0;2;0;2;5;5;5;5;5];
commands = actionListNum';
%commands = [0, 0, 0, 0, 0, 0, 0, 0, 0];

% 0 = fist hand gesture
% 2 = bunny hand gesture
% 5 = high five hand gesture
pose_map = containers.Map({0, 2, 5}, {standing, sitting, squat});


% Notice, transition time is a calcuation of 6/(numel(commands)*(6+7)) for
% 6 steps and 7/(numel(commands)*(6+7)) steps where a step is
% tf-t0/length(steps)

command = pose_map(commands(1));
command2 = pose_map(commands(2));
transition = pose_transition(command, command2, 6); %move transition
t2 = pose_transition(command2, command2, 7); %wait transition
transition = cat_transitions(transition, t2);
command = command2;

for idx = 3:numel(commands)
    cmd_num = commands(idx);
    command2 = pose_map(cmd_num);
    t2 = pose_transition(command, command2, 6); %move transition % Originally 6
    transition = cat_transitions(transition, t2);
    t2 = pose_transition(command2, command2, 7); %wait transition
    transition = cat_transitions(transition, t2);
    command = command2;
end

BRUpperLeg.signals.values=transition.BRUpper;
BLUpperLeg.signals.values=transition.BLUpper;

FRUpperLeg.signals.values=transition.FRUpper;
FLUpperLeg.signals.values=transition.FLUpper;

BRLowerLeg.signals.values=transition.BRLower;
BLLowerLeg.signals.values=transition.BLLower;

FRLowerLeg.signals.values=transition.FRLower;
FLLowerLeg.signals.values=transition.FLLower;

steps = numel(transition.BRUpper);

t = linspace(0, simTime, steps)'; 

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
