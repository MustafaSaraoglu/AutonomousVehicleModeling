% Initialisation of VehicleFollowing.slx model

%% Constraints
s_min = 0;
s_max = 1000;

sDot_min = 0;
sDot_max = 30;

sDDot_min = -3;
sDDot_max = 2;

theta_max = pi/4; % maximum steering angle

%% Road
w_lane = 3.7; % lane width
roadTrajectory =    [s_min  0   0;
                     s_max  0   0;
                     0      0   0;
                     0      0   0];

% Double lane
d_min = - w_lane/2;
d_max = 3/2*w_lane;         

%% V1 - Lead
% Vehicle's geometry
V1_dim = [6; 2];  % length and width
R1 = sqrt((V1_dim(1)/2)^2 + (V1_dim(2)/2)^2); 

% Initial values
X1_0 = 40;
V1_0 = 20;

v1_ref = 10;

% Constant values
Y1 = 0;
Yaw1 = 0;

%% V2- Ego
% Vehicle's geometry
V2_dim = [4; 2];  % length and width
% Radius of circle around rectangle vehicle representation for collision 
% detection
R2 = sqrt((V1_dim(2)/2)^2 + (V2_dim(2)/2)^2); 

% Initial values
X2_0 = 0;
Y2_0 = 0;
Yaw2_0 = 0;

V2_0 = 15;

%% Lane Changing maneuver
% Time for lane changing and overtaking
deltaT_LC = 5;
deltaT_OT = 5;

% Look ahead distance for pure pursuit
d_lookAhead = 6;