%
% Produce an array of data that represents a transition from pose A to pose
% B given the 8 beginning angles A, 8 ending angles, and transition time t
% e.g. changes from sitting to standing in t seconds.

% t is structured as t =  = 0.2 * [0:49]'; where 0.2 is the time step and
% there are 50 steps;

function angle_set = pose_transition(pose_start, pose_end, t)
    angle_set = struct;
    fields = fieldnames(pose_start); % == pose_end)
    for joint = 1:numel(fields)
        angle_start = pose_start.(fields{joint});
        angle_end = pose_end.(fields{joint});
        % Algorithm for closest angle transition over 0,2*pi border
        poss_angle_ends = [angle_end-2*pi, angle_end, angle_end+2*pi]; % Possible angles to transition to
        abs_poss_angle_ends = [abs(angle_end-2*pi - angle_start), ...
            abs(angle_end - angle_start), abs(angle_end+2*pi - angle_start)];
        [~, index] = min(abs_poss_angle_ends); % Argmin
        angle_end = poss_angle_ends(index);
        
        steps = numel(t);
        angle_steps = linspace(angle_start, angle_end, steps);
        angle_set.(fields{joint}) = angle_steps';
    end
end