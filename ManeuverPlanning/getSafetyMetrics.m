%% Use this script as a post simulation script to get the safety metrics
% Metrics: TTC (Time-to-collision), TET (Time exposed TTC)
% Input: out (logged data from the simulation)

clear TTC TET;

TTC = [TTCright(out.d_other, out.s_other, out.d, out.s, out.v, out.v_other); ...
    TTCleft(out.d_other, out.s_other, out.d, out.s, out.v, out.v_other)];

minTTC = 0;
meanTTC = 0;

TET = getTET(TTC);
TET_val = 0;
% Output Min TTC

disp(['Min TTC is:', num2str(minTTC)])
% Output Mean TTC
disp(['Min TTC is:', num2str(meanTTC)])
% Output TET
disp(['TET is:', num2str(TET_val)])

function TTC_left = TTCleft(d_other, s_other, d, s, v, v_other)
% find the index of other vehicle on the left lane
left = d_other>=2.85;
idx_left_lane = find(left == 1);

% the longitudinal displacement of the other vehicles on the left
% lane
s_other = s_other(:,idx_left_lane);
% the index of s of ego vehicle on the left lane
idx_ego_left_lane = d >= 2.85 ;
% the longitudinal displacement of ego vehicle on the left lane
s_ego_left = s.*idx_ego_left_lane;
% the longitudinal displacement of other vehicles on the left lane
s_other_left = s_other.*idx_ego_left_lane;
% index of leader vehicle on the same lane as ego
idx_leader_s = (s_other_left - s_ego_left) > 0.01;
% the speed of ego vehicle on the left lane
v_ego_left = v.*idx_ego_left_lane;
% the speed of other vehicles on the same lane
v_other_left = v_other(:,idx_left_lane).*idx_ego_left_lane;
idx_slow_v = (v_ego_left-v_other_left)>0.01;
% calculate the longitudinal displacement and speed differenz at the same
% time step
s_left = (s_other_left - s_ego_left).*idx_leader_s.*idx_slow_v;
v_left = (v_ego_left-v_other_left).*idx_leader_s.*idx_slow_v;
TTC_left = s_left./v_left;
TTC_left = TTC_left';

dim1 = size(TTC_left, 1);
dim2 = size(TTC_left, 2);
for j = 1:dim1
    for i = 1:dim2
        if isnan(TTC_left(j,i))
            TTC_left(j,i) = inf;
        end
    end
end

end


function TTC_right = TTCright(d_other, s_other, d, s, v, v_other)
% find the index of other vehicle on the right lane
right = d_other <= 0.1;
idx_right_lane = find(right == 1);
% the longitudinal displacement of the other vehicles on the right lane
s_other = s_other(:,idx_right_lane);
% the index of s of ego vehicle on the right lane
idx_ego_right_lane = d <= 0.1;
% the longitudinal displacement of ego vehicle on the right lane
s_ego_right = s.*idx_ego_right_lane;
% the longitudinal displacement of other vehicles on the right lane
s_other_right = s_other.*idx_ego_right_lane;
% index of leader vehicle on the same lane as ego
idx_leader_s = (s_other_right - s_ego_right) > 0.01; %exclude zero values
% the speed of ego vehicle on the right lane
v_ego_right = v.*idx_ego_right_lane;
% the speed of other vehicles on the same lane
v_other_right = v_other(:,idx_right_lane).*idx_ego_right_lane;
idx_slow_v = (v_ego_right-v_other_right)>0.01;
% calculate the longitudinal displacement and speed differenz at the same
% time step
s_right = (s_other_right - s_ego_right).*idx_leader_s.*idx_slow_v;
v_right = (v_ego_right-v_other_right).*idx_leader_s.*idx_slow_v;
% TTC
TTC_right = s_right./v_right;
TTC_right = TTC_right';
% assign inf if the component of TTC is NaN
dim1 = size(TTC_right, 1);
dim2 = size(TTC_right, 2);
for j = 1:dim1
    for i = 1:dim2
        if isnan(TTC_right(j,i))
            TTC_right(j,i) = inf;
        end
    end
end
end

function TET = getTET(TTC)
% TET calculation
% index of TET
t = 1;
delta_t = 0.05;
TET = zeros(1,10);
T =10;
% find the index of noninfinity components
nonzeroIdx = find(TTC(4,:) ~= Inf);
for i=1:length(nonzeroIdx)
    % if i is the length of nonzeroIdx or the differenz between (i+1)th and
    % ith componnent is 1, then add the nonzeroIdx(i)th component of TTC to
    % beta(t)
    if i == length(nonzeroIdx)|| nonzeroIdx(i+1) - nonzeroIdx(i) == 1
        if TTC(1,nonzeroIdx(i)) < T
            TET(t) = TET(t) + TTC(1,nonzeroIdx(i));
        end
    else
        % otherwise increase t and add the nonzeroIdx(i)th component of TTC
        % to beta(t)
        if TTC(1,nonzeroIdx(i)) < T
            TET(t) = TET(t) + TTC(1,nonzeroIdx(i));
        end
                 nonzeroIdx(i)
                 nonzeroIdx(i+1)
        t = t+1;
    end
end
TET = TET*delta_t;

end





