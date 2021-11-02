% Initialisation of VehicleFollowing.slx model

%% Constraints
s_min = 0; % Minimum allowed longitudinal postion [m]
s_max = 1000; % Maximum allowed longitudinal postion [m]

v_min = 0; % Minimum allowed longitudinal velocity [m/s]
v_max = 30; % Maximum allowed longitudinal velocity [m/s]

a_min = -3; % Minimum allowed longitudinal acceleration [m/s^2]
a_max = 2; % Maximum allowed longitudinal acceleration [m/s^2]

steerAngle_max = pi/4; % Maximum allowed steering angle [rad]
angularVelocity_max = 0.1; % Maximum angular velocity [rad/s]

%% Road
laneWidth = 3.7; % lane width [m]

% Road trajectory according to MOBATSim map format
% Straight Road
% roadTrajectory =    [s_min  0   0;
%                      s_max  0   -200;
%                      0      0   0;
%                      0      0   0];
                 
% Curved Road
roadTrajectory =    [s_min  0        0;
                     s_max  0       -s_max;
                     pi/2   s_min   -s_max;
                     -1     -1      -1]; % TODO: '+1' or '-1' ?; '-' seems to be counter clockwise

% Double lane
d_min = -laneWidth/2; % Right lateral lane boundary [m]
d_max = 3/2*laneWidth; % Left lateral lane boundary [m]       

%% Leading Vehicle
% Vehicle's geometry
dimensionsLead = [6; 2];  % Length and width [[m]; [m]]
wheelBaseLead = 4; % Wheel base [m]
% Radius of circle around rectangle vehicle representation for collision 
% detection [m]
radiusLead = sqrt((dimensionsLead(1)/2)^2 + (dimensionsLead(2)/2)^2); 

sLead_0 = 40; % Initial Frenet s-coordinate [m]
dLead = 0; % (Initial) Frenet d-coordinate [m]

vLead_0 = 10; % Initial longitudinal velocity [m/s]

vLead_ref = vLead_0; % Reference longitudinal velocity [m/s]

%% Ego Vehicle
% Vehicle's geometry
dimensionsEgo = [4; 2];  % length and width [[m]; [m]]
wheelBaseEgo = 3; % Wheel base [m]
% Radius of circle around rectangle vehicle representation for collision 
% detection [m]
radiusEgo = sqrt((dimensionsEgo(1)/2)^2 + (dimensionsEgo(2)/2)^2); 

sEgo_0 = 0; % Initial Frenet s-coordinate [m]
dEgo_0 = 0; % Initial Frenet d-coordinate [m]

% Transformation to Cartesian for Bicycle Kinematic Model
% xEgo_0: Initial x-coordinate [m]
% yEgo_0: Initial y-coordinate [m]
% yawEgo_0: Initial steering angle [rad]
[positionEgo_0, yawEgo_0] = Frenet2Cartesian(0, [sEgo_0, dEgo_0], roadTrajectory);
xEgo_0 = positionEgo_0(1);
yEgo_0 = positionEgo_0(2);

vEgo_0 = 15; % Initial longitudinal velocity [m/s]

%% Lateral Control
% Lane Changing maneuver
durationToLeftLane = 5; % Time for lane changing (to left lane) [s]
durationToRightLane = 5; % Time for overtaking (to right lane) [s]

numberWaypoints = 15; % Number of waypoints to provide for Pure Pursuit
lookAheadDistance = 6; % Look ahead distance for Pure Pursuit [m]

Ts = 0.05; % Sampling time [s]
timeHorizon = 2; % Time horizon for trajectory genereation [s]