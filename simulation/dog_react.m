% dog_react.m

% Culmination of the Target Goal:
% The Dog should respond to any command indefinitely

% This version has 4 commands: stand, sit, walk
% The 4 gesture commands are fist, bunny, high five, and blank

% Pre-loading the walking work

%% Mechanical parameters
L = 0.54;    % Leg length (m)
g = 9.81;   % Acceleration due to gravity (m/s^2)
m = 28.4;     % Mass (Kg)
k = 5315;   % Leg stiffness (N/m)

%% Initial conditions positions and speeds
x0 = 0.0;   % Horizontal position of mass in middle of stance phase
y0 = 0.85*L;  % Height of mass in middle of stance phase
u0 = 1.5;   % Horizontal speed in middle of stance phase (/s)
v0 = 0.0;   % Vertical speed in the middle of the stance phase (/s)

%% Additional top level parameters
M = m*0.9;     %mass of payload
R = 100;       %radius of mass support (set high to prevent accidental rotation)
L_stance = y0; %Leg length at mid stance
L_back = 0.72; %Length of spine (m)
joint_damping = 0.1; %Damping of all joints (Nm/s), standard damping
th1 = 25*pi/180;     %initial angle of upper-leg to vertical, multiple options (chosen due to ease)
th2_retracted = 45*pi/180; %Angle of knee retraction restriction



sim('sm_gait_selection') %Input: mechanical parameters and additional conditions   Outputs: Leg angle and energy parameters


%% Derived parameters

l1 = 0.27; %length of upper leg
l2 = 0.27; %length of lower leg
th2 = 2*asin((L/2)/l1); %Initial inside knee angle
m1 = 2/(l1+l2); 
m2 = 2/(l1+l2); %Leg mass per unit length (m1 - upper, m2 - lower)

%Joint angles at mid-stance
dif_Ls = L-L_stance; %difference from leg length at midstance to full length of leg
th2_mid = 2*asin(0.5*(L-dif_Ls)/l1); %knee angle at midstance
th1_mid = pi/2 - th2_mid/2; %upper leg angle at midstance

k2 = k*dif_Ls*l1*sin(th1_mid)/(th2-th2_mid); %knee spting at mid stance based on stiffness values asssumed earlier

h0 = 0; %initial height (on ground)
hdot0 = 0; %intial vertical velocity (none)


%% Calculate arial phase trajectory by calculating acceleration using leg
%andgle and velocity as initial condition
T_gait = leg_angle.time(end); %Gait Period
idx_stance = find(stance.signals.values<=0.5,1); 
T_stance = 2*stance.time(idx_stance);
accel_aerial = (-leg_angle0-(T_gait-T_stance/2)*leg_vel0)/(0.5*(T_gait-T_stance/2)^2);

sim('sm_aerial_trajectory') %input: aerial accelration, gait period, midstance period   Output: acceleration, velocity and angle of joints


%% Build full trajectory in order to build hip and knee torques using inverse kinematics later
%This data collection section was heavily derived from Mathworks code

% Second half of stance
idx_second = find(leg_angle.signals.values(2:end)>=0, 1 );
data1 = leg_angle.signals.values(1:idx_second-1);
time1 = leg_angle.time(1:idx_second-1);

%Aerial 
data2 = aerial_phase_angle.signals.values;
time2 = leg_angle.time(idx_second) + aerial_phase_angle.time;

%First half of stance
idx_first = 1+find(leg_angle.signals.values(2:end)>0, 1 );
data3 = leg_angle.signals.values(idx_first:end);
time3 = time2(end) + leg_angle.time(idx_first:end) - leg_angle.time(idx_first-1);

TimeValues = [time1;time2;time3];
DataValues = [data1;data2;data3];


% set_param('sm_inv_kin','SimMechanicsOpenEditorOnUpdate','off'); % Turn off automatic Mechanics Explorer
simParameters = struct;
simParameters.SimMechanicsOpenEditorOnUpdate = 'off';
simOut = sim('sm_inv_kin', simParameters); %input: angles, gait period, geomatry, time and data values    ouptut: knee and hip angles, velocity, torque

UpperLegAngle = simOut.UpperLegAngle;
LowerLegAngle = simOut.LowerLegAngle;

%% Solve for one leg

ULAngleValues = deg2rad(UpperLegAngle.Data);
LLAngleValues = deg2rad(LowerLegAngle.Data);

ULAngleValues2 = circshift(ULAngleValues,floor(length(ULAngleValues)/2));
LLAngleValues2 = circshift(LLAngleValues,floor(length(LLAngleValues)/2));

%% Setting up the static poses
% One Walk Transition
walk = struct;
walk.BRUpper = ULAngleValues;
walk.BLUpper = ULAngleValues2;

walk.FRUpper = ULAngleValues2;
walk.FLUpper = ULAngleValues;

walk.BRLower = LLAngleValues;
walk.BLLower = LLAngleValues2;

walk.FRLower = LLAngleValues2;
walk.FLLower = LLAngleValues;

% Starting Walking Pose
walk_start = struct;
walk_start.BRUpper = ULAngleValues(1);
walk_start.BLUpper = ULAngleValues2(1);

walk_start.BRLower = LLAngleValues(1);
walk_start.BLLower = LLAngleValues2(1);

walk_start.FRUpper = ULAngleValues2(1);
walk_start.FLUpper = ULAngleValues(1);

walk_start.FRLower = LLAngleValues2(1);
walk_start.FLLower = LLAngleValues(1);

% Ending Walking Pose
walk_end = struct;
walk_end.BRUpper = ULAngleValues(end);
walk_end.BLUpper = ULAngleValues2(end);

walk_end.BRLower = LLAngleValues(end);
walk_end.BLLower = LLAngleValues2(end);

walk_end.FRUpper = ULAngleValues2(end);
walk_end.FLUpper = ULAngleValues(end);

walk_end.FRLower = LLAngleValues2(end);
walk_end.FLLower = LLAngleValues(end);


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

% Crouch Constants
squat = struct;
squat.BRUpper = deg2rad(180-135);
squat.BLUpper = deg2rad(180-135);
squat.BRLower = deg2rad(90+180);
squat.BLLower = deg2rad(90+180);
squat.FRUpper = deg2rad(180-225);
squat.FLUpper = deg2rad(180-225);
squat.FRLower = deg2rad(270+180);
squat.FLLower = deg2rad(270+180);

% Sitting Constant
sitting = struct;
sitting.FRUpper = deg2rad(162.823146-180);
sitting.FLUpper = deg2rad(162.823146-180);
sitting.FRLower = deg2rad(180-59.610529);
sitting.FLLower = deg2rad(180-59.610529);
sitting.BRUpper = deg2rad(180-139.25);
sitting.BLUpper = deg2rad(180-139.25);
sitting.BRLower = deg2rad(0);
sitting.BLLower = deg2rad(0);


%% Setting up the Commands
% actionListNum = [0;0;2;2;5;5;5;5;5];
% actionListNum = [0;2;5];
% commands = actionListNum';
prompt = "Woof woof, I'm CHIP! (commands: ) ";
commands = input(prompt);
commands = repelem(commands,4);
disp(commands')

% This version has 4 commands: stand, sit, walk, crouch
% The 4 gesture commands are fist, bunny, high five, and blank

% 9 = high five hand gesture -> Squat
% 0 = fist hand gesture -> Stand
% 2 = bunny hand gesture -> Sit
% 5 = high five hand gesture -> Walking
pose_map = containers.Map({0, 2, 9}, {standing, sitting, squat});

n = length(commands); % Number of commands

% Hard Code Note: Each Command takes 624 steps, The shortest walk

% Go through commands to build out transitions
if commands(1) == 5 % Walking
    sim_movement = walk;
    prev_command = walk_end;
elseif commands(1) == 2 % Sitting
    sim_movement = pose_map(commands(1));
    prev_command = pose_map(commands(1));
elseif commands(1) == 0 % Standing
    sim_movement = pose_map(commands(1));
    prev_command = pose_map(commands(1));
elseif commands(1) == 9 % Crouching
    sim_movement = pose_map(commands(1));
    prev_command = pose_map(commands(1));
end
    

for i = 2:n
    if commands(i) == 5 % Do one Walk
        if commands(i-1) == 5 % If prev walking, continue
            sim_movement = cat_transitions(sim_movement, walk);
            prev_command = walk_end;
        elseif commands(i-1) == 2 % If sitting, first stand, then walk
            transition = pose_transition(prev_command, pose_map(0), 300); % Stand up
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = pose_map(0);
            command = walk_start;
            transition = pose_transition(prev_command, command, 300); % from stand to walk
            sim_movement = cat_transitions(sim_movement, transition);
            sim_movement = cat_transitions(sim_movement, walk); % Walk once
            prev_command = walk_end;
        elseif commands(i-1) == 0 % If standing, transition out
            command = walk_start;
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            sim_movement = cat_transitions(sim_movement, walk); % Walk once
            prev_command = walk_end;
        elseif commands(i-1) == 9 % If sitting, first stand, then walk
            transition = pose_transition(prev_command, pose_map(0), 300); % Stand up
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = pose_map(0);
            command = walk_start;
            transition = pose_transition(prev_command, command, 300); % from stand to walk
            sim_movement = cat_transitions(sim_movement, transition);
            sim_movement = cat_transitions(sim_movement, walk); % Walk once
            prev_command = walk_end;
        end
    elseif commands(i) == 2 % Do one Sit
        if commands(i-1) == 5 % If prev walking, Transition to
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 524); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 100); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 2 % If sitting, continue
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 0 % If standing, transition out
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 9 % If squat, transition out
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        end
    elseif commands(i) == 0 % Do one stand
        if commands(i-1) == 5 % If prev walking, Transition to
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 2 % If sitting, continue
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 0 % If standing, transition out
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 9 % If squat, transition out
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        end
    elseif commands(i) == 9 % Do one stand
        if commands(i-1) == 5 % If prev walking, Transition to
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 9 % If squat, continue
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 0 % If standing, transition out
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        elseif commands(i-1) == 2 % If squat, transition out
            command = pose_map(commands(i));
            transition = pose_transition(prev_command, command, 324); % Transition
            sim_movement = cat_transitions(sim_movement, transition);
            transition = pose_transition(command, command, 300); % Wait
            sim_movement = cat_transitions(sim_movement, transition);
            prev_command = command;
        end
    end
    
end

%% Load Angles and Run
BRUpperLeg.signals.values=sim_movement.BRUpper;
BLUpperLeg.signals.values=sim_movement.BLUpper;

FRUpperLeg.signals.values=sim_movement.FRUpper;
FLUpperLeg.signals.values=sim_movement.FLUpper;

BRLowerLeg.signals.values=sim_movement.BRLower;
BLLowerLeg.signals.values=sim_movement.BLLower;

FRLowerLeg.signals.values=sim_movement.FRLower;
FLLowerLeg.signals.values=sim_movement.FLLower;

simTime = (10) * sum(commands == 5) + ...
1 * sum( commands== 2) + ...
1 * sum( commands== 0) + ...
1 * sum( commands== 9);
simParameters = struct;
simParameters.StopTime = num2str(simTime); % units in seconds

steps = numel(sim_movement.BRUpper);
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
