% Initialisation of VehicleFollowing.slx model

%% Constraints
s_min = 0; % Minimum allowed longitudinal postion [m]
s_max = 1000; % Maximum allowed longitudinal postion [m]

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
roadTrajectory =    [s_min  0   0;
                     s_max  0   0;
                     0      0   0;
                     0      0   0];
               
% % Curved Road
% roadTrajectory =    [s_min  0        0;
%                      s_max  0       -s_max;
%                      pi/2   s_min   -s_max;
%                      -1     -1      -1]; 

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

sLead_0 = 60; % Initial Frenet s-coordinate [m]
dLead = 0; % (Initial) Frenet d-coordinate [m]

% Transformation to Cartesian for 3D-Animation
% xLead_0: Initial x-coordinate [m]
% yLead_0: Initial y-coordinate [m]
% yawLead_0: Initial steering angle [rad]
[positionLead_0, yawLead_0] = Frenet2Cartesian(sLead_0, dLead, roadTrajectory);
xLead_0 = positionLead_0(1);
yLead_0 = positionLead_0(2);

vLead_0 = 10; % Initial longitudinal velocity [m/s]

vLead_ref = vLead_0; % Reference longitudinal velocity [m/s]

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

%% Lateral Control
% Hardcoded conditions
isOccupiedLeft = false;

% Lane Changing maneuver
durationToLeftLane = 5; % Time for lane changing (to left lane) [s]
durationToRightLane = 5; % Time for overtaking (to right lane) [s]

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

if timeHorizon < durationToLeftLane || timeHorizon < durationToRightLane
    error('Time horizon must be greater than or equal to duration to change lane.');
end

%% Space Discretisation
cell_length = 5; % Cell length in s-coordinate [m]
laneCell_width = 3; % Width of right/left lane cell [m]
[spaceDiscretisation, spaceDiscretisationMatrix] = discretiseContinuousSpace(roadTrajectory, laneWidth, cell_length, laneCell_width); % Discretisation of continuous space

%% Functions
function [spaceDiscretisation, spaceDiscretisationMatrix] = discretiseContinuousSpace(roadTrajectory, laneWidth, cell_length, laneCell_width)
% Divide road into discrete cells using Frenet Coordinates

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
    number_columns = 5; % Divide road width into 5 parts
    spaceDiscretisation = cell(number_rows, number_columns); % Store cell array for comprehensibility
    % Using 2*rows x 2*columns matrix instead of rows x columns (2x2) cell array because it is much more efficient than cell array
    spaceDiscretisationMatrix = zeros(2*number_rows, 2*number_columns); 

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
            spaceDiscretisationMatrix(2*row-1:2*row, 2*column-1:2*column) = [s_start, s_end; d_start, d_end]; % Index shift because of matrix structure
        end
    end
end