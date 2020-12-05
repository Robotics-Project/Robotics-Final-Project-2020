%Kinematics Walking Simulation
%Creates a simulation of a walking 4-legged robot using the simulink model
%formed for this project and directly calculated kinematics

%The basis of this simulation is based off the pose simulation our team
%conducted earlier in the project

%In addition this simulation also takes from the simscape structure of 
%the Mathworks code "Running Robot Model in Simscape" available here:
%https://www.mathworks.com/matlabcentral/fileexchange/64237-running-robot-model-in-simscape

%NOTE: Kinematics models represented in these simscape files are extremly
%similar to those explored for the individual pose simulations, however
%the Mathworks model takes into account parameters from outside the scope
%of this course. As such, these simscape models have been modified for the
%purposes of this project.


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

sim('sm_inv_kin') %input: angles, gait period, geomatry, time and data values    ouptut: knee and hip angles, velocity, torque


%% 
% Find starting and finishing indices for aerial phase for one leg
idx1 = find(hip_torque.time > T_gait/4, 1 );
idx2 = find(hip_torque.time > 7*T_gait/4, 1 );

% Hip angle in aerial phase
t_th = Hip.time(idx1+1:idx2-1);
t_th = t_th - t_th(1);
u_th = Hip.signals.values(idx1+1:idx2-1);
th_dot0 = (u_th(2)-u_th(1))/(t_th(2)-t_th(1));

% Hip velocity in aerial phase
t_w = HipVel.time(idx1+1:idx2-1);
t_w = t_w - t_w(1);
u_w = HipVel.signals.values(idx1+1:idx2-1);

% Maximum inertia about hip
J = m2*l2*l1^2+(m1*l1^3+m2*l2^3)/3;


%% The interface to hold and input walking equations for each step
%to simulations

% kin_walk = struct;
% kin_walk.BRUpper = 0;
% kin_walk.BLUpper = 0;
% kin_walk.BRLower = 0;
% kin_walk.BLLower = 0;
% kin_walk.FRUpper = 0;
% kin_walk.FLUpper = 0;
% kin_walk.FRLower = 0;
% kin_walk.FLLower = 0;
% 
% 
% attempts = 1; % Number of simulations to run
% steps = 200; % 100 time steps
% tic
% 
% for joint = 1:numel(fields)
%     
%     
% end