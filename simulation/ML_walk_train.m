%
% Training ML Algorithm to get robot dog to walk
% Assumptions:

% Generational Machine Learning Used: Good Walking dogs will pass its
% characteristics to the children in the next generation

% Dog will attempt to walk for only 20 seconds
% There will be 30 attempts per generation

% Dogs will be scored
% A very positive score for a good walk is moving the farthest forward and
% keeping the torso correctly aligned (belly face down, back face up)
% A penalty with would introduce if the torso is not correctly aligned
% throughout the walk
% The top 10 best scores are the parents for the next generation


% 1. Start with randomized walking data
% Have 30 different randomizations to start

walk = struct;
walk.BRUpper = 0;
walk.BLUpper = 0;
walk.BRLower = 0;
walk.BLLower = 0;
walk.FRUpper = 0;
walk.FLUpper = 0;
walk.FRLower = 0;
walk.FLLower = 0;
fields = fieldnames(walk);

% Random walk data
steps = 200; % 100 time steps

for joint = 1:numel(fields)
    random_walk = zeros(steps, 1);
    random_walk(1) = 0;

    for i=2:steps
        R = rand(1);
        if R < 0.5
            S = random_walk(i-1) + 0.1;
        else
            S = random_walk(i-1) - 0.1;
        end
        random_walk(i) = S;
    end
    walk.(fields{joint}) = random_walk;
end



% 2. Simulate each walk, return the torso and distance data. Score the walk
% according to torso data and distance data.
simTime = 20;
simParameters = struct;
simParameters.StopTime = num2str(simTime); % units in seconds
BRUpperLeg.signals.values=walk.BRUpper;
BLUpperLeg.signals.values=walk.BLUpper;

FRUpperLeg.signals.values=walk.FRUpper;
FLUpperLeg.signals.values=walk.FLUpper;

BRLowerLeg.signals.values=walk.BRLower;
BLLowerLeg.signals.values=walk.BLLower;

FRLowerLeg.signals.values=walk.FRLower;
FLLowerLeg.signals.values=walk.FLLower;

steps = numel(walk.BRUpper);

t = linspace(0, simTime, steps)'; 

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

distance_forward = simOut.distance_forward.data;
axis = simOut.axis.data;
angle = simOut.q.data;

% Axis-Angle Output and Distance Forward (X direction)
figure(1);
plot(simOut.axis.time, simOut.axis.data);
title("axis of rotation");

figure(2);
plot(simOut.q.time, simOut.q.data);
title("angle of rotation");

figure(3);
plot(simOut.distance_forward.time, simOut.distance_forward.data);
title("distance forward");

% 3. Format the walking input, and the torso and distance output into one
% matrix. Then put the top ten scored matrices together


% 4. Feed the input matrix into the ML algorithm to generate 30 new walks
% for the simulation to try.

% Repeat 2-4


