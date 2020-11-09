% poses_sim
% The interface file to hold the pose equations for the joints of the robot
% simulation

% Simulation Parameters for model
simParameters = struct;
simParameters.StopTime = '10'; % units in seconds
t = linspace(0, 10, 100)'; 

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


commands = [0, 1, 2, 0, 1];

pose_map = containers.Map({0, 1, 2}, {standing, sitting, squat});

command = pose_map(commands(1));
command2 = pose_map(commands(2));
transition = pose_transition(command, command2, 12); %move transition
t2 = pose_transition(command2, command2, 13); %wait transition
transition = cat_transitions(transition, t2);
command = command2;

for idx = 3:numel(commands)
    cmd_num = commands(idx);
    command2 = pose_map(cmd_num);
    t2 = pose_transition(command, command2, 12); %move transition
    transition = cat_transitions(transition, t2);
    t2 = pose_transition(command2, command2, 13); %wait transition
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


simOut = sim('robot.slx', simParameters);