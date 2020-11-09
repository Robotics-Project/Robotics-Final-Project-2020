%
% Concatenate transitions

function timeline = cat_transitions(transition1, transition2)
    fields = fieldnames(transition1);
    for joint = 1:numel(fields)
        t1 = transition1.(fields{joint});
        t2 = transition2.(fields{joint});
        if t1(end) ~= t2(1)
            diff = t2(1) - t1(end);
            t2 = t2 - diff;
        end
        timeline.(fields{joint}) = cat(1, t1, t2);
    end
end