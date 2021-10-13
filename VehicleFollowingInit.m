% Initialisation of VehicleFollowing.slx model

%% Road
w_lane = 3.7; % lane width
roadTrajectory =    [0      0   0;
                     1000   0   0;
                     0      0   0;
                     0      0   0];

%% Constraints
s_min = 0;
s_max = 1000;

% Double lane
d_min = - w_lane/2;
d_max = 3/2*w_lane;

sDot_min = 0;
sDot_max = 30;

sDDot_min = -3;
sDDot_max = 2;

theta_max = pi/4; % maximum steering angle

%% V1 - Lead
% Vehicle properties
V1_dim = [6; 2];  % length and width

% Initial values
X1_0 = 40;
V1_0 = 20;

v1_ref = 10;

% Constant values
Y1 = 0;
Yaw1 = 0;

%% V2- Ego
% Vehicle properties
V2_dim = [4; 2];  % length and width

% Initial values
X2_0 = 0;
Y2_0 = 0;
Yaw2_0 = 0;

V2_0 = 15;

%% Lane Changing maneuver
% Time for lane changing and overtaking
deltaT_LC = 5;
deltaT_OT = 5;