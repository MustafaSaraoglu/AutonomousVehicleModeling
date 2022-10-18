function prepare_simulation(options)
% Prepare simulation
    
    %% Default values
    arguments
        options.road             (4,3) double =  [0      0   0; % Road trajectory according to 
                                                  2000   0   0; % MOBATSim map format
                                                  0      0   0;
                                                  0      0   0]; 
         
        options.n_other         (1,1) double = 2;               % Number of other vehicles
                                              
        % Structure:[dataEgoVehicle, dataOtherVehicle1, ..., dataOtherVehicleN]
        % Kinematic initial conditions
        options.s_0             (1,:) double = [0, 60, 40];  % Initial Frenet s-coordinate [m]
        options.d_0             (1,:) double = [0, 0, 3.7];  % (Initial) Frenet d-coordinate [m]
        options.v_0             (1,:) double = [20, 10, 13]; % Initial longitudinal velocity [m/s]
        options.v_ref           (1,:) double = [20, 10, 13]; % Reference longitudinal velocity [m/s]       
        
        options.planner         (1,1) string = 'Minimax-Dev';     % Mode of discrete planner
                                % Alternative = 'RuleBased'
                                % Alternative = 'POMDP'
                                % Alternative = 'Minimax-Fast'
        
        options.lateral         (1,1) string = 'STANLEY';    % Mode of lateral control
                               % Alternative = 'PURE_PURSUIT'
        options.Th              (1,1) double = 5;            % Time horizon for trajectory 
                                                             % genereation [s]
        
        options.sigmaS          (1,1) double = 0.2;          % Standard deviation for 
                                                             % measuring other vehicles' 
                                                             % s-coordinate [m]
        options.sigmaV          (1,1) double = 0.2;          % Standard deviation for 
                                                             % measuring other vehicles' 
                                                             % speeds [m/s]
        options.visualizeTree       (1,1) logical = false;       % By default dont visualize the decision trees
    end
    
    %% Road
    road.laneWidth = 3.7; % lane width [m]
    road.trajectory =  options.road;
    
    %% Ego Vehicle
     % Vehicle's geometry
    ego.dimensions = [4; 2]; % Length and width [[m]; [m]]
    ego.wheelBase = 3; % Wheel base [m]
    
    ego.s_0 = options.s_0(1); 
    ego.d_0 = options.d_0(1); 
    ego.v_0 = options.v_0(1); 
    ego.v_ref = options.v_ref(1); 

    % Transformation to Cartesian for Bicycle Kinematic Model
    % ego.x_0: Initial x-coordinate [m]
    % ego.y_0: Initial y-coordinate [m]
    % ego.yaw_0: Initial steering angle [rad]
    [ego.position_0, ego.yaw_0] = Frenet2Cartesian(ego.s_0, ego.d_0, road.trajectory);
    ego.x_0 = ego.position_0(1);
    ego.y_0 = ego.position_0(2);
    
    %% Other Vehicles
     % Vehicles' geometry
    other.number = options.n_other;
    other.dimensions = [4; 2].*ones(2, other.number);
    other.wheelBase = 3*ones(1, other.number); 
    
    other.s_0 = options.s_0(2:end); 
    other.d_0 = options.d_0(2:end);
    other.v_0 = options.v_0(2:end);
    other.v_ref = options.v_ref(2:end); 

    % Transformation to Cartesian for 3D-Animation
    % other.x_0: Initial x-coordinate [m]
    % other.y_0: Initial y-coordinate [m]
    % yawLead_0: Initial steering angle [rad]
    [other.position_0, other.yaw_0] = Frenet2Cartesian(other.s_0', other.d_0', road.trajectory);
    other.x_0 = other.position_0(:, 1)';
    other.y_0 = other.position_0(:, 2)';
    other.yaw_0 = other.yaw_0';
    
    %% Planner
    planner.mode = options.planner;
    
    %% Lateral Control
    lateral.mode = options.lateral;
    
    % Gains for PID controller
    lateral.Kp = 8.7639;  
    lateral.Ki = 9.3465;
    lateral.Kd = 0.1412;

    lateral.numberWaypoints = 15; % Number of waypoints to provide for Pure Pursuit
    lateral.lookAheadDistance = 6; % Look ahead distance for Pure Pursuit [m]

    lateral.forwardMotionGain = 1.6684; % Position gain of forward motion for Stanley
    
    %% Trajectory Generation
    Ts = 0.01; % Sample time [s] for trajectory generation

    trajectoryGeneration.timeHorizon = options.Th;
    trajectoryGeneration.partsTimeHorizon = 5; % Divide time horizon into partsTimeHorizon 
                                               % equal parts
    
    %% Uncertainty
    uncertainty.sigmaS = options.sigmaS;
    uncertainty.sigmaV = options.sigmaV;
    
    %% Constraints
    constraints.v_min = 0; % Minimum allowed longitudinal velocity [m/s]
    constraints.v_max = 30; % Maximum allowed longitudinal velocity [m/s]

    constraints.a_min = -3; % Minimum allowed longitudinal acceleration [m/s^2]
    constraints.a_max = 2; % Maximum allowed longitudinal acceleration [m/s^2]
    constraints.a_emergency = -5; % Longitudinal acceleration for emergency brake  [m/s^2]

    constraints.steerAngle_max = pi/4; % Maximum allowed steering angle [rad]
    constraints.angularVelocity_max = 0.1; % Maximum angular velocity [rad/s]
    
    %% Space Discretisation
    discreteCells.cell_length = 5; % Cell length in s-coordinate [m]
    discreteCells.laneCell_width = 3; % Width of right/left lane cell [m]
    % Discretisation of continuous space
    spaceDiscretisation = discretiseContinuousSpace(road.trajectory, road.laneWidth, ...
                                                    discreteCells.cell_length, ...
                                                    discreteCells.laneCell_width); 
    
    %% Assign Variables in Base-Workspace
    assignin('base', 'visualizeTree', options.visualizeTree); 
    assignin('base', 'road', road); 
    assignin('base', 'ego', ego); 
    assignin('base', 'other', other); 
    assignin('base', 'planner', planner); 
    assignin('base', 'lateral', lateral); 
    assignin('base', 'trajectoryGeneration', trajectoryGeneration); 
    assignin('base', 'lateral', lateral); 
    assignin('base', 'Ts', Ts); 
    assignin('base', 'constraints', constraints); 
    assignin('base', 'uncertainty', uncertainty); 
    assignin('base', 'discreteCells', discreteCells); 
    assignin('base', 'spaceDiscretisation', spaceDiscretisation); 
    assignin('base', 'nextState', 'FreeDrive'); % Start with FreeDrive
    
    %% Disable Warning: 'Property Unspecified Default Value'
    id = 'SystemBlock:MATLABSystem:ParameterWithUnspecifiedDefaultValueRestrictedToBuiltinDataType';
    warning('off',id);
    
    %% Load Model
    modelName = 'ManeuverPlanning';
    load_system(modelName);
    open_system(modelName);
     
    %% Set Planner Mode
    % Set Variant Subsystem
    set_param('ManeuverPlanning/Ego Vehicle/Discrete Planner', 'modePlanner', planner.mode);
    
    %% Set Lateral Mode
    % Set Variant Subsystem
    set_param('ManeuverPlanning/Ego Vehicle/Lateral Control', 'modeLateral', lateral.mode);
    
end

