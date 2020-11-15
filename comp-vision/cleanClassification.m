load classifedEx;
len = size(gestureClassified);

% use class array and count array
prev = "";
count = [0];
class = [char(prev)];
i = 0;
for idx=1:len(1)
    if gestureClassified(idx,:) == prev
        count(i) = count(i) + 1;
    else
        prev = gestureClassified(idx,:);
        i = i + 1;
        class(i,:) = prev;
        count(i) = 1;
    end
end
disp(class);
disp(count);

cntThresh = 10;
actionList = class(count>cntThresh,:);

disp(actionList);
actionListNum = actionList(:,2)-'0';
disp(actionListNum);