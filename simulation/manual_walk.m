% manual_walk.m
% The main run file for seeing the model walk from a manual algorithm

% Simulation Parameters for model
simTime = 0.80*(10 + 3);
simParameters = struct;
simParameters.StopTime = num2str(simTime); % units in seconds

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

% TimeValues = [time1;time2;time3];
DataValues = [data1;data2;data3];

% Then is the upper leg only of angles

% Repeat Limb Movement n times
DataValues = repmat(DataValues, 10, 1);

% Wait in position at the end for n Limb Movements;
wait_pad = zeros(3*length(DataValues), 1);
wait_pad(wait_pad == 0) = DataValues(end);

DataValues = [DataValues; wait_pad];

walk.BRUpper = DataValues;
walk.BLUpper = DataValues;

walk.FRUpper = DataValues;
walk.FLUpper = DataValues;

%% Load Angles and Run
BRUpperLeg.signals.values=walk.BRUpper;
BLUpperLeg.signals.values=walk.BLUpper;

FRUpperLeg.signals.values=walk.FRUpper;
FLUpperLeg.signals.values=walk.FLUpper;


steps = numel(walk.BRUpper);
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

simOut = sim('robot.slx', simParameters);

% % Axis-Angle Output and Distance Forward (X direction)
% figure(1);
% plot(simOut.axis.time, simOut.axis.data);
% legend("X", "Y", "Z");
% xlabel("Time (Seconds)");
% title("axis of rotation");
% 
% figure(2);
% plot(simOut.q.time, simOut.q.data);
% ylabel("Angle (radians)");
% xlabel("Time (Seconds)");
% title("angle of rotation");
% 
% figure(3);
% plot(simOut.distance_forward.time, simOut.distance_forward.data);
% title("distance forward");
% xlabel("Time (Seconds)");
% ylabel("Distance (meters?)");