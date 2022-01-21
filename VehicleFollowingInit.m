% Initialisation of VehicleFollowing.slx model

%% Road
road.laneWidth = 3.7; % lane width [m]

% Road trajectory according to MOBATSim map format
% Straight Road
road.trajectory =    [0      0   0;
                      1000   0   0;
                      0      0   0;
                      0      0   0];
               
% % Curved Road
% road.trajectory =    [0        0        0;
%                       1000     0       -1000;
%                       pi/2     0       -1000;
%                       -1       -1      -1];  

%% Ego Vehicle
%% Tunable:
% Vehicle's geometry
ego.dimensions = [4; 2];  % length and width [[m]; [m]]
ego.wheelBase = 3; % Wheel base [m]

% Kinematic initial conditions
ego.s_0 = 0; % Initial Frenet s-coordinate [m]
ego.d_0 = 0; % Initial Frenet d-coordinate [m]

ego.v_0 = 20; % Initial longitudinal velocity [m/s]

ego.v_ref = ego.v_0 ; % Reference longitudinal velocity [m/s]

%% Non-Tunable:
% Transformation to Cartesian for Bicycle Kinematic Model
% ego.x_0: Initial x-coordinate [m]
% ego.y_0: Initial y-coordinate [m]
% ego.yaw_0: Initial steering angle [rad]
[ego.position_0, ego.yaw_0] = Frenet2Cartesian(ego.s_0, ego.d_0, road.trajectory);
ego.x_0 = ego.position_0(1);
ego.y_0 = ego.position_0(2);

%% Other Vehicles
% Structure:[dataVehicle1, dataVehicle2, ..., dataVehicleN]
%% Tunable
% Vehicles' geometry
other.dimensions = [6, 4; 2, 2];  % Length and width [[m]; [m]]
other.wheelBase = [4, 3]; % Wheel base [m]

% Kinematic initial conditions
other.s_0 = [60, 40]; % Initial Frenet s-coordinate [m]
other.d_0 = [0, road.laneWidth]; % (Initial) Frenet d-coordinate [m]

other.v_0 = [10, 13]; % Initial longitudinal velocity [m/s]

other.v_ref = other.v_0 ; % Reference longitudinal velocity [m/s]

%% Non-Tunable:
% Transformation to Cartesian for 3D-Animation
% other.x_0: Initial x-coordinate [m]
% other.y_0: Initial y-coordinate [m]
% yawLead_0: Initial steering angle [rad]
[other.position_0, other.yaw_0] = Frenet2Cartesian(other.s_0', other.d_0', road.trajectory);
other.x_0 = other.position_0(:, 1)';
other.y_0 = other.position_0(:, 2)';
other.yaw_0 = other.yaw_0';

%% Constraints
constraints.v_min = 0; % Minimum allowed longitudinal velocity [m/s]
constraints.v_max = 30; % Maximum allowed longitudinal velocity [m/s]

constraints.a_min = -3; % Minimum allowed longitudinal acceleration [m/s^2]
constraints.a_max = 2; % Maximum allowed longitudinal acceleration [m/s^2]
constraints.a_emergency = -5; % Longitudinal acceleration for emergency brek [m/s^2]

constraints.steerAngle_max = pi/8; % Maximum allowed steering angle [rad]
constraints.angularVelocity_max = 0.1; % Maximum angular velocity [rad/s]

%% Trajectory Generation
Ts = 0.01; % Sample time [s] for trajectory generation

trajectoryGeneration.timeHorizon = 5; % Time horizon for trajectory genereation [s]
trajectoryGeneration.partsTimeHorizon = 3; % Divide time horizon into partsTimeHorizon equal parts

%% Lateral Control
% Gains for PID controller
PID.Kp = 8.7639; 
PID.Ki = 9.3465;
PID.Kd = 0.1412;

PurePursuit.numberWaypoints = 15; % Number of waypoints to provide for Pure Pursuit
PurePursuit.lookAheadDistance = 6; % Look ahead distance for Pure Pursuit [m]

Stanley.forwardMotionGain = 1.6684; % Position gain of forward motion for Stanley

%% Space Discretisation
discreteCells.cell_length = 5; % Cell length in s-coordinate [m]
discreteCells.laneCell_width = 3; % Width of right/left lane cell [m]
spaceDiscretisation = discretiseContinuousSpace(road.trajectory, road.laneWidth, discreteCells.cell_length, discreteCells.laneCell_width); % Discretisation of continuous space

%% Disable Warning: 'Property Unspecified Default Value'
id = 'SystemBlock:MATLABSystem:ParameterWithUnspecifiedDefaultValueRestrictedToBuiltinDataType';
warning('off',id);