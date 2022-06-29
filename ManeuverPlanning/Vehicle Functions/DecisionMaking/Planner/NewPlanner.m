classdef NewPlanner < matlab.System & handle & matlab.system.mixin.Propagates & ...
        matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    %NEWPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Nontunable)
        
        % Assigned to EGO INFO
        timeHorizon             % Time horizon for trajectory genereation [s]
        partsTimeHorizon        % Divide time horizon into equal parts  
        Ts                      % Sample time [s]    
        minimumAcceleration     % Minimum longitudinal acceleration [m/s^2]
        maximumAcceleration     % Maximum longitudinal acceleration [m/s^2]
        emergencyAcceleration   % Acceleration for emergency break [m/s^2]
        maximumVelocity         % Maximum allowed longitudinal velocity [m/s]               
        wheelBase               % Wheel base vehicle [m]
        steerAngle_max          % Maximum steering angle [rad]
        vEgo_ref                % Reference velocity for ego vehicle [m/s]
        
        % Assigned to OTHERS INFO
        vOtherVehicles_ref      % Reference speed for other vehicles [m/s]
        sigmaS                  % Standard deviation for measuring other vehicles' s-coordinate [m]
        sigmaV                  % Standard deviation for measuring other vehicles' speeds [m/s]
             
        % Assigned to ROAD INFO
        LaneWidth               % Width of road lane [m]
        RoadTrajectory          % Road trajectory according to MOBATSim map format
        spaceDiscretisation     % Space Discretisation
        

    end
    
    % Pre-computed constants
    properties(Access = protected)
        Ego     % All the constants regarding the ego vehicle
        Others  % All the constants regarding the other vehicles
        Road    % All the constants regarding the road information
        
        d_destination % Reference lateral destination (right or left lane)
        isChangingLane % Return if currently executing lane changing maneuver
        t_ref % Variable to store a specific simulation time of interest
        toleranceReachLane % Accepted tolerance to reach destination lane
        
        drivingModes % Possible driving modes
        states % Possible driving states
        currentState % Current driving state
        previousState % Previous driving state
        
        SearchTree % Search tree to find best decision
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.Ego     = EgoInfo(obj.Ts,obj.timeHorizon,obj.partsTimeHorizon,obj.minimumAcceleration,obj.maximumAcceleration,obj.emergencyAcceleration,obj.maximumVelocity,obj.wheelBase,obj.steerAngle_max,obj.vEgo_ref);
            obj.Others  = OthersInfo(obj.vOtherVehicles_ref, obj.sigmaS, obj.sigmaV);
            obj.Road    = RoadInfo(obj.RoadTrajectory, obj.LaneWidth, obj.spaceDiscretisation);
            
            obj.d_destination       = 0; % Start on right lane
            obj.toleranceReachLane  = 0.05;
            obj.isChangingLane      = false;
            obj.t_ref               = 0;

            % Create a trajectory generator
            NewTrajectoryGenerator = NewTrajectoryGeneration(obj.Ego, obj.Road.RoadTrajectory);
            
            % Get all possible Maneuvers
            obj.drivingModes = NewManeuver.getallActions(NewTrajectoryGenerator);
            
            % Create a maneuver planner with all the possible maneuvers
            ManeuverPlanner = ...
                NewManeuverPlanner(obj.drivingModes,obj.Ego,obj.Road,obj.Others,NewTrajectoryGenerator);
            
            % Create a search tree object using the Maneuver Planner
            obj.SearchTree = NewTreeSearch(obj.Ego, ManeuverPlanner);
            
            stateNames = {...
                % Keep Lane
                'FreeDrive', 1;
                'VehicleFollowing', 2;
                'EmergencyBrake', 3;
                
                % LaneChanging
                'LaneChanging', 4;
                };
            obj.states = containers.Map(stateNames(:, 1)', [stateNames{:, 2}]);
            
            % Initial state: Free Drive and on the right lane
            obj.currentState = 1;
            disp('@t=0s: Initial state is: ''FreeDrive''.');
        end
        
        
        
        function [changeLaneCmd, drivingMode] = stepImpl(obj, poseEgo, poseOtherVehicles, ...
                speedsOtherVehicles, vEgo)
            % Necessary to return some output even if there is no command
            changeLaneCmd = 0;
            
            % Return lane change command and the current driving mode
            
            [sEgo, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]);
            orientationEgo = poseEgo(3);
            
            obj.previousState = obj.currentState;
            

            
            if obj.isChangingLane && (abs(dEgo - obj.d_destination) < 0.05)
                obj.isChangingLane = false;
            end
            
            % Only check everey timeHorizon/partsTimeHorizon seconds because expensive operation
            % and if not changing lane
            if get_param('ManeuverPlanning', 'SimulationTime') >= obj.t_ref && ~obj.isChangingLane
                obj.t_ref = get_param('ManeuverPlanning', 'SimulationTime') + ...
                    obj.Ego.timeHorizon/obj.Ego.partsTimeHorizon;
                
                % Define vehicle states
                currentState_Ego = State(sEgo, dEgo, orientationEgo, vEgo);
                
                [sOther, dOther] = Cartesian2Frenet(obj.RoadTrajectory, ...
                    [poseOtherVehicles(1, :)', ...
                    poseOtherVehicles(2, :)']);
                
                % Preallocation
                currentStates_Other(1, size(poseOtherVehicles, 2)) = State([], [], [], []);
                for id_other = 1:size(poseOtherVehicles, 2)
                    currentStates_Other(id_other) = State(sOther(id_other), dOther(id_other), ...
                        poseOtherVehicles(3, id_other), ...
                        speedsOtherVehicles(id_other));
                end
                
                % TODO: Free Drive if no vehicle in front on right lane for faster computation?
                
                [bestDecision_Ego, dG] = ...
                    obj.SearchTree.iterativeMinimax(currentState_Ego, currentStates_Other, ...
                    obj.d_destination);
                
                %% Plot the tree:
                %                 figure(2);
                %                 dG_iteration = dG{2};
                %                 plot(dG_iteration, 'EdgeLabel',...
                %                 dG_iteration.Edges.Power, 'EdgeColor',...
                %                 cell2mat(dG_iteration.Edges.Color), 'NodeColor',...
                %                 cell2mat(dG_iteration.Nodes.Color), 'Layout', 'layered');
                %%
                nextDecision = bestDecision_Ego(1);
                description_nextDecision = strsplit(nextDecision.description, '_');
                nextState = description_nextDecision{1};
                obj.currentState = obj.states(nextState);
                
                if strcmp(nextState, 'LaneChanging')
                    obj.isChangingLane = true;
                    
                    % Round: avoid very small value differences
                    % Use either 0 or 3.7
                    obj.d_destination = round(nextDecision.futureState.d, 1);
                    
                    T_LC = extractBetween(description_nextDecision{2}, '{T', '}');
                    changeLaneCmd = str2double(T_LC{1});
                end
            end
            
            %drivingMode = obj.getStateInfo(obj.currentState);
            drivingMode = obj.drivingModes{obj.currentState}.id;
            
            obj.displayNewState(obj.currentState, obj.previousState);
        end
        
        
        function displayNewState(obj, currentState, previousState)
            % Display state name if switched to another state
            
            if currentState ~= previousState
                newState = currentState;
                name_newState = class(obj.drivingModes{obj.currentState});
                
                t = get_param('ManeuverPlanning', 'SimulationTime');
                
                fprintf('@t=%fs: Switched to State: ''%s''.\n', t, name_newState);
            end
        end
        
        
        
        function [out1, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];
            
            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out1, out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            
            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end
        
        function [out1, out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            
            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end
        
        function [out1, out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            
            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end

