% Initialisation of VehicleFollowing.slx model

%% Constraints
v_min = 0; % Minimum allowed longitudinal velocity [m/s]
v_max = 30; % Maximum allowed longitudinal velocity [m/s]

a_min = -3; % Minimum allowed longitudinal acceleration [m/s^2]
a_max = 2; % Maximum allowed longitudinal acceleration [m/s^2]
a_emergency = -5; % Longitudinal acceleration for emergency brek [m/s^2]

steerAngle_max = pi/8; % Maximum allowed steering angle [rad]
angularVelocity_max = 0.1; % Maximum angular velocity [rad/s]

%% Road
laneWidth = 3.7; % lane width [m]

% Road trajectory according to MOBATSim map format
% Straight Road
roadTrajectory =    [0      0   0;
                     1000   0   0;
                     0      0   0;
                     0      0   0];
               
% % Curved Road
% roadTrajectory =    [0        0        0;
%                      1000     0       -1000;
%                      pi/2     0       -1000;
%                      -1       -1      -1];  

%% Ego Vehicle
% Vehicle's geometry
dimensionsEgo = [4; 2];  % length and width [[m]; [m]]
wheelBaseEgo = 3; % Wheel base [m]
% Radius of circle around rectangle vehicle representation for collision 
% detection [m]
radiusEgo = sqrt((dimensionsEgo(1)/2)^2 + (dimensionsEgo(2)/2)^2); 

minimumTurningRadiusEgo = wheelBaseEgo/tan(steerAngle_max); % Minimum turning radius [m]

sEgo_0 = 0; % Initial Frenet s-coordinate [m]
dEgo_0 = 0; % Initial Frenet d-coordinate [m]

% Transformation to Cartesian for Bicycle Kinematic Model
% xEgo_0: Initial x-coordinate [m]
% yEgo_0: Initial y-coordinate [m]
% yawEgo_0: Initial steering angle [rad]
[positionEgo_0, yawEgo_0] = Frenet2Cartesian(sEgo_0, dEgo_0, roadTrajectory);
xEgo_0 = positionEgo_0(1);
yEgo_0 = positionEgo_0(2);

vEgo_0 = 20; % Initial longitudinal velocity [m/s]

vEgo_ref = vEgo_0; % Reference longitudinal velocity [m/s]

%% Other Vehicles
% [dataVehicle1, dataVehicle2, ..., dataVehicleN]

% Vehicles' geometry
dimensionsOtherVehicles = [6, 4; 2, 2];  % Length and width [[m]; [m]]
wheelBaseOtherVehicles = [4, 3]; % Wheel base [m]
% Radius of circle around rectangle vehicle representation for collision 
% detection [m]
radiusOtherVehicles = sqrt((dimensionsOtherVehicles(1, :)/2).^2 + (dimensionsOtherVehicles(2, :)/2).^2);

sOtherVehicles_0 = [60, 40]; % Initial Frenet s-coordinate [m]
dOtherVehicles = [0, laneWidth]; % (Initial) Frenet d-coordinate [m]

% Transformation to Cartesian for 3D-Animation
% xLead_0: Initial x-coordinate [m]
% yLead_0: Initial y-coordinate [m]
% yawLead_0: Initial steering angle [rad]
[positionOtherVehicles_0, yawOtherVehicles_0] = Frenet2Cartesian(sOtherVehicles_0', dOtherVehicles', roadTrajectory);
xOtherVehicles_0 = positionOtherVehicles_0(:, 1)';
yOtherVehicles_0 = positionOtherVehicles_0(:, 2)';
yawOtherVehicles_0 = yawOtherVehicles_0';

vOtherVehicles_0 = [10, 13]; % Initial longitudinal velocity [m/s]

vOtherVehicles_ref = vOtherVehicles_0; % Reference longitudinal velocity [m/s]

%% Lateral Control
isAcceptedTrajectory = false; % Check whether lane changing trajectory is accepted

numberWaypoints = 15; % Number of waypoints to provide for Pure Pursuit
lookAheadDistance = 6; % Look ahead distance for Pure Pursuit [m]

Ts = 0.01; % Sampling time [s]
timeHorizon = 5; % Time horizon for trajectory genereation [s]
partsTimeHorizon = 3; % Divide time horizon into partsTimeHorizon equal parts

% Gains for PID controller
Kp = 8.7639; 
Ki = 9.3465;
Kd = 0.1412;

forwardMotionGain = 1.6684; % Position gain of forward motion for Stanley

%% Space Discretisation
cell_length = 5; % Cell length in s-coordinate [m]
laneCell_width = 3; % Width of right/left lane cell [m]
spaceDiscretisation = discretiseContinuousSpace(roadTrajectory, laneWidth, cell_length, laneCell_width); % Discretisation of continuous space

%% Functions
function spaceDiscretisation = discretiseContinuousSpace(roadTrajectory, laneWidth, cell_length, laneCell_width)
% Divide road into discrete cells using Frenet Coordinates 
% return cell array containing the corner points of each discrete cell
% represented by the corresponding row and column index in the cell array

    route = roadTrajectory([1, 2],[1, 3]).*[1, -1; 1, -1];
    radian = roadTrajectory(3, 1);
    startPoint = route(1, :);

    if radian == 0 % Straight road
        endPoint = route(2, :);
        route_Vector = endPoint - startPoint;

        routeLength = norm(route_Vector);
    else % Curved road
        rotationCenter = roadTrajectory(3, [2, 3]).*[1, -1]; 
        startPointVector = startPoint - rotationCenter;
        routeRadius = norm(startPointVector); 

        routeLength = abs(radian*routeRadius);
    end

    % Cell dimensions
    roadBoundryCell_width = (laneWidth - laneCell_width)/2; % Width of road boundry cells [m]
    roadCenterCell_width = 2*roadBoundryCell_width; % Width of road center cell [m]

    number_rows = ceil(routeLength/cell_length);
    number_columns = 5; % Divide road width into 5 cells
    spaceDiscretisation = cell(number_rows, number_columns); % Preallocate spaceDiscretisation cell array

    for row = 1:number_rows
        s_start = (row-1)*cell_length;
        s_end = s_start + cell_length;
        if row == number_rows % Check last row in case could not divide perfectly into equal sized cells
            s_end = routeLength;
        end

        for column = 1:number_columns
            switch column
                case 1 % Right road boundry cell
                    d_start = -laneWidth/2; % Start at -laneWidth/2
                    d_end = -laneWidth/2 + roadBoundryCell_width;
                case 2 % Right lane cell
                    d_start = -laneWidth/2 + roadBoundryCell_width;
                    d_end = -laneWidth/2 + roadBoundryCell_width + laneCell_width;
                case 3 % Center of the road cell
                    d_start = -laneWidth/2 + roadBoundryCell_width + laneCell_width;
                    d_end = -laneWidth/2 + roadBoundryCell_width + laneCell_width + roadCenterCell_width;
                case 4 % Left lane cell
                    d_start = -laneWidth/2 + roadBoundryCell_width + laneCell_width + roadCenterCell_width;
                    d_end = -laneWidth/2 + roadBoundryCell_width + 2*laneCell_width + roadCenterCell_width;
                case 5 % Left road boundry cell
                    d_start = -laneWidth/2 + roadBoundryCell_width + 2*laneCell_width + roadCenterCell_width;
                    d_end = -laneWidth/2 + 2*roadBoundryCell_width + 2*laneCell_width + roadCenterCell_width;
            end

            spaceDiscretisation{row, column} = [s_start, s_end; d_start, d_end];
        end
    end
end