% Initialisation of VehicleFollowing.slx model

%% Constraints
s_min = 0; % Minimum allowed longitudinal postion [m]
s_max = 1000; % Maximum allowed longitudinal postion [m]

sDot_min = 0; % Minimum allowed longitudinal velocity [m/s]
sDot_max = 30; % Maximum allowed longitudinal velocity [m/s]

sDDot_min = -3; % Minimum allowed longitudinal acceleration [m/s^2]
sDDot_max = 2; % Maximum allowed longitudinal acceleration [m/s^2]

theta_max = pi/4; % Maximum allowed steering angle [rad]

%% Road
w_lane = 3.7; % lane width [m]

% Road trajectory according to MOBATSim map format
roadTrajectory =    [s_min  0   0;
                     s_max  0   0;
                     0      0   0;
                     0      0   0];

% Double lane
d_min = - w_lane/2; % Right lateral lane boundary [m]
d_max = 3/2*w_lane; % Left lateral lane boundary [m]       

%% V1 - Lead
% Vehicle's geometry
V1_dim = [6; 2];  % length and width [[m]; [m]]
% Radius of circle around rectangle vehicle representation for collision 
% detection [m]
R1 = sqrt((V1_dim(1)/2)^2 + (V1_dim(2)/2)^2); 

X1_0 = 40; % Initial x-coordinate [m]

Y1 = 0; % (Initial) y-coordinate [m]
Yaw1 = 0; % (Initial) steering angle [rad]

V1_0 = 20; % Initial longitudinal velocity [m/s]

v1_ref = 10; % Reference longitudinal velocity [m/s]

%% V2- Ego
% Vehicle's geometry
V2_dim = [4; 2];  % length and width [[m]; [m]]
% Radius of circle around rectangle vehicle representation for collision 
% detection [m]
R2 = sqrt((V1_dim(2)/2)^2 + (V2_dim(2)/2)^2); 

X2_0 = 0; % Initial x-coordinate [m]
Y2_0 = 0; % Initial y-coordinate [m]
Yaw2_0 = 0; % Initial steering angle [rad]

V2_0 = 15; % Initial longitudinal velocity [m/s]

%% Lateral Control
% Modes for lateral control
PURE_PURSUIT = Simulink.Variant('MODE == 1'); % Mode for Pure Pursuit
LATERAL_SPEED = Simulink.Variant('MODE == 2'); % Mode for computing steering command using lateral speed
STANLEY = Simulink.Variant('MODE == 3'); % Mode for Stanley

% By default use MODE 1 when opening the model
MODE = 1;

% Lane Changing maneuver
deltaT_LC = 5; % Time for lane changing [s]
deltaT_OT = 5; % Time for overtaking [s]

d_lookAhead = 6; % Look ahead distance for Pure Pursuit [m]