classdef LocalTrajectoryPlanner < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
% LocalTrajectoryPlanner Superclass for generating necessary inputs for 
% the lateral controllers.
    
    properties(Nontunable)
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        durationToLeftLane % Time for lane changing [s]
        durationToRightLane % Time for overtaking [s]
        timeHorizon % Time horizon for trajectory genereation [s]
        partsTimeHorizon % Divide time horizon into partsTimeHorizon equal parts
        Ts % Sampling time for trajectory generation [s]
    end
    
    % Pre-computed constants
    properties(Access = protected)
        % Coefficents for lane changing trajectory
        a0
        a1
        a2
        a3
        a4
        a5
           
        trajectoryReferenceLength % Number of points for trajectory generation
        
        residualLaneChangingTrajectory % Store residual lane changing trajectory if timeHorizon is to small to fit whole trajectory
        
        maneuvers % Possible maneuvers
        
        lanes % Possible lane states
        
        laneChangeCmds % Possible commands for lane changing

        currentTrajectoryFrenet % Planned trajectory for maneuver in Frenet coordinates [s, d, time]
        
        fractionTimeHorizon % Fraction of time horizon when divided into partsTimeHorizon equal parts
        counter % Counter to stop at correct simulation time
        predictedTrajectory % Store future trajectory predictions for replanning
    end
    
    methods
        function obj = LocalTrajectoryPlanner(varargin)
            %WAYPOINTGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.laneChangeCmds = ...
                containers.Map({'CmdFollow', 'CmdStartToLeft', 'CmdStartToRight'}, [0, 1, -1]);
            
            obj.lanes = ...
                containers.Map({'RightLane', 'ToLeftLane', 'LeftLane', 'ToRightLane'}, [0, 0.5, 1, -0.5]);
            
            % +1 because planned trajectory contains current Waypoint at current time
            obj.trajectoryReferenceLength = obj.timeHorizon/obj.Ts + 1; 
            
            obj.currentTrajectoryFrenet = zeros(obj.trajectoryReferenceLength, 3);
            
            obj.residualLaneChangingTrajectory = [];
            
            obj.fractionTimeHorizon = obj.timeHorizon/obj.partsTimeHorizon;
            obj.counter = 0;
            obj.predictedTrajectory = [];
        end
        
        function trajectoryFrenet = planFrenetTrajectory(obj, changeLaneCmd, replan, currentLane, s, d, currentOrientation, v_average)
        % Plan trajectory for the next obj.timeHorizon seconds in Frenet coordinates
            
            lengthDifference = 0; % Difference in length between current trajectory and reference trajectory
            if changeLaneCmd 
                laneChangingTrajectoryFrenet = obj.calculateLaneChangingManeuver(changeLaneCmd, s, d, v_average); 
                lengthDifference = obj.divideLaneChangingTrajectory(laneChangingTrajectoryFrenet);
            end
            
            if replan
                [~, roadOrientation] = Frenet2Cartesian(s, d, obj.RoadTrajectory);
                d_dot = v_average*tan(currentOrientation - roadOrientation); % TODO: Recheck formula
                d_destination = obj.getLateralDestination(currentLane);
                
                obj.currentTrajectoryFrenet = obj.reCalculateTrajectory(s, d, d_dot, d_destination, obj.timeHorizon, v_average);
                obj.setTrajectoryPrediction();
            end
            
            obj.updateCurrentTrajectory(s, currentLane, v_average, lengthDifference);
            
            trajectoryFrenet = obj.currentTrajectoryFrenet;
        end
        
        function lengthDifference = divideLaneChangingTrajectory(obj, laneChangingTrajectory)
        % Divide provided lane changing trajectory into current trajectory and residual lane changing trajectory 
        % and return the difference in length between the new current trajectory and the reference trajectory
        
            lengthDifference = obj.trajectoryReferenceLength - size(laneChangingTrajectory, 1);
            if lengthDifference < 0 
                % Divide into current trajectory and residual lane changing trajectory
                obj.currentTrajectoryFrenet = laneChangingTrajectory(1:obj.trajectoryReferenceLength, :);
                obj.residualLaneChangingTrajectory = laneChangingTrajectory(obj.trajectoryReferenceLength+1:end, :);
                lengthDifference = 0; 
            else
                obj.currentTrajectoryFrenet = laneChangingTrajectory;
            end
        end
        
        function updateCurrentTrajectory(obj, s_current, currentLane, v_average, lengthDifference)
        % Remove passed waypoints and add new ones, always keep the same trajectory length
            
            ID_current = sum(s_current >= obj.currentTrajectoryFrenet(:, 1));
        
            if (ID_current > 1) || lengthDifference 
                obj.currentTrajectoryFrenet = obj.currentTrajectoryFrenet(ID_current:end, :);
                pointsToAdd = ID_current - 1 + lengthDifference;
                
                addTrajectory = obj.calculateTrajectoryToAdd(currentLane, v_average, pointsToAdd);

                obj.currentTrajectoryFrenet = [obj.currentTrajectoryFrenet; addTrajectory];
            end
            
            if size(obj.currentTrajectoryFrenet, 1) ~= obj.trajectoryReferenceLength 
                error('Trajectory length is incorrect'); % For debugging
            end
        end
        
        function addTrajectory = calculateTrajectoryToAdd(obj, currentLane, v_average, pointsToAdd)
        % Calculate trajectory that needs to be added to the current trajectory in order to get the correct reference trajectory length 
            
            s_lastElement = obj.currentTrajectoryFrenet(end, 1);
            t_lastElement = obj.currentTrajectoryFrenet(end, 3);
            d_destination = obj.getLateralDestination(currentLane);
        
            if isempty(obj.residualLaneChangingTrajectory)
                durationToAdd = pointsToAdd*obj.Ts;
                addTrajectory = obj.calculateStraightTrajectory(s_lastElement, t_lastElement, d_destination, v_average, durationToAdd);
            elseif pointsToAdd >= size(obj.residualLaneChangingTrajectory, 1)
                addTrajectory = obj.residualLaneChangingTrajectory;

                durationToAdd = (pointsToAdd - size(obj.residualLaneChangingTrajectory, 1))*obj.Ts;

                addTrajectory = [addTrajectory; obj.calculateStraightTrajectory(s_lastElement, t_lastElement, d_destination, v_average, durationToAdd)];
                obj.residualLaneChangingTrajectory = [];
            else
                addTrajectory = obj.residualLaneChangingTrajectory(1:pointsToAdd, :);
                obj.residualLaneChangingTrajectory = obj.residualLaneChangingTrajectory(pointsToAdd+1:end ,:);
            end
        end
        
        function laneChangingTrajectoryFrenet = calculateLaneChangingManeuver(obj, changeLaneCmd, s, d, v_average)
        % Calculate the lane changing maneuver either to the left or right lane
            
            if changeLaneCmd == obj.laneChangeCmds('CmdStartToLeft')
                d_destination = obj.LaneWidth;
                durationManeuver = obj.durationToLeftLane;
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStartToRight')
                d_destination = 0;
                durationManeuver = obj.durationToRightLane;
            end
            laneChangingTrajectoryFrenet = obj.calculateLaneChangingTrajectory(s, d, d_destination, durationManeuver, v_average);
        end
        
        function laneChangingTrajectoryFrenet = calculateLaneChangingTrajectory(obj, s_current, d_currnet, d_destination, durationManeuver, v_average)
        % Calculate minimum jerk trajectory for lane changing maneuver
            
            % Initial conditions
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current 
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % d_destination
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [d_currnet; 0; 0; d_destination; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
            
            % Calculate trajectory for whole maneuver
            t_discrete = 0:obj.Ts:durationManeuver; 
            
            s_trajectory = s_current + v_average*t_discrete;
            d_trajectory = obj.a0 + obj.a1*t_discrete + obj.a2*t_discrete.^2 + obj.a3*t_discrete.^3 + obj.a4*t_discrete.^4 + obj.a5*t_discrete.^5;
            time = get_param('VehicleFollowing', 'SimulationTime') + t_discrete;
            
            laneChangingTrajectoryFrenet = [s_trajectory', d_trajectory', time'];
        end 
        
        function newTrajectoryFrenet = reCalculateTrajectory(obj, s_current, d_currnet, d_dot_current, d_destination, durationManeuver, v_average)
        % Recalculate minimum jerk trajectory 
            
            % Initial conditions
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current 
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; % d_dot_current
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % d_destination
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [d_currnet; d_dot_current; 0; d_destination; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
            
            % Calculate trajectory for whole maneuver
            t_discrete = 0:obj.Ts:durationManeuver; 
            
            s_trajectory = s_current + v_average*t_discrete;
            d_trajectory = obj.a0 + obj.a1*t_discrete + obj.a2*t_discrete.^2 + obj.a3*t_discrete.^3 + obj.a4*t_discrete.^4 + obj.a5*t_discrete.^5;
            time = get_param('VehicleFollowing', 'SimulationTime') + t_discrete;
            
            newTrajectoryFrenet = [s_trajectory', d_trajectory', time'];
        end  
        
        function straightTrajectoryFrenet = calculateStraightTrajectory(obj, s_last, t_last, d_destination, v_average, duartion)
        % Calculate straight trajectory staying on the same lane
        
            t_discrete = obj.Ts:obj.Ts:duartion; 
            s_trajectory = s_last + v_average*t_discrete;
            d_trajectory = d_destination*ones(1, length(t_discrete));
            time = t_last + t_discrete;
            
            straightTrajectoryFrenet = [s_trajectory', d_trajectory', time']; 
        end
        
        function currentTrajectoryCartesian = getCurrentTrajectoryCartesian(obj)
        % Return current trajectory in Cartesian coordinates [x, y, time]
            
            [currentTrajectoryCartesianNoTimeStamps, ~] = Frenet2Cartesian(obj.currentTrajectoryFrenet(:, 1), obj.currentTrajectoryFrenet(:, 2), obj.RoadTrajectory);
            time = obj.currentTrajectoryFrenet(:, 3);
            currentTrajectoryCartesian = [currentTrajectoryCartesianNoTimeStamps, time];
        end
        
        function [s_ref, d_ref] = getNextFrenetTrajectoryWaypoints(obj, s, numberWPs)
        % Get the next waypoint(s) for current trajectory according to current s in Frenet coordinates
        
            ID_nextWP = sum(s >= obj.currentTrajectoryFrenet(:, 1)) + 1;
            
            numberWPsMax = size(obj.currentTrajectoryFrenet(ID_nextWP:end, 1), 1);
            if (numberWPs <= 0) || (numberWPs > numberWPsMax)
                fprintf('Number of Waypoints must be greater than 0 and smaller than %d \n', numberWPsMax);
                error('Number of waypoints is not valid');
            end
            
            s_ref = obj.currentTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberWPs-1), 1);
            d_ref = obj.currentTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberWPs-1), 2);
        end
        
        function replan = calculateTrajectoryError(obj, s, d)
        % Track error between predicted trajectory and actual trajectory every fractionTimeHorizon seconds
        % If the error exceeds threshold, replan maneuver
           
            replan = false;
            if get_param('VehicleFollowing', 'SimulationTime') > obj.counter*obj.fractionTimeHorizon
                if ~isempty(obj.predictedTrajectory)
                    error_s_d = obj.predictedTrajectory(1:2) - [s, d]; 
                    replan = (abs(error_s_d(1)) > 1) || (abs(error_s_d(2)) > 0.1);
                end 
                
                obj.setTrajectoryPrediction();
                obj.counter = obj.counter + 1;
            end
        end
        
        function setTrajectoryPrediction(obj)
        % Set prediction for the future position according to the planned trajectory in the next fractionTimeHorizon seconds
            
            ID_nextPrediction = sum((obj.counter+1)*obj.fractionTimeHorizon  >= obj.currentTrajectoryFrenet(:, 3)); % ID for next predicted time
            obj.predictedTrajectory = obj.currentTrajectoryFrenet(ID_nextPrediction, :); 
        end
        
        function d_destination = getLateralDestination(obj, currentLane)
        % Get the reference lateral destination (either right or left lane)
        
            d_destination = 0;
            if currentLane == obj.lanes('ToLeftLane') || currentLane == obj.lanes('LeftLane')
                d_destination = obj.LaneWidth;
            end
        end
        
        function laneCenterReached = isReachedDestinationLane(obj, currentLane)
        % Return if the vehicle has reached the destination lane
            
            laneCenterReached = false;
            if currentLane == obj.lanes('RightLane') || currentLane == obj.lanes('LeftLane')
                laneCenterReached = true;
            end    
        end
        
%         function trajectoryToPlot = getTrajectoryToPlot(obj, trajectoryCartesian, currentLane)
%         % Return trajectory in Cartesian coordinates with reduced samples for plotting  
%             
%             % TODO: Find a way to output variable size
%             if obj.isReachedDestinationLane(currentLane)
%                 trajectoryToPlot = [trajectoryCartesian(1, 1:2); trajectoryCartesian(end, 1:2)]; % For straight line two points are sufficient
%             else
%                 trajectoryToPlot = [trajectoryCartesian(1:obj.timeHorizon*10:end, 1:2); trajectoryCartesian(end, 1:2)]; % Reduce points to plot
%             end
%         end
    end
end

