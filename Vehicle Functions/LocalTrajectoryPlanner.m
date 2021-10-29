classdef LocalTrajectoryPlanner < CoordinateTransformations
% LocalTrajectoryPlanner Superclass for generating necessary inputs for 
% the lateral controllers.
    
    properties(Nontunable)
        durationToLeftLane % Time for lane changing [s]
        durationToRightLane % Time for overtaking [s]
        timeHorizon % Time horizon for trajectory genereation [s]
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

        currentTrajectoryFrenet % Planned trajectory for maneuver in Frenet coordinates [s, d, dDot]
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
            
            obj.trajectoryReferenceLength = obj.timeHorizon/obj.Ts;
            
            obj.currentTrajectoryFrenet = zeros(obj.timeHorizon/obj.Ts, 3);
            
            obj.residualLaneChangingTrajectory = [];
        end
        
        function trajectoryFrenet = planTrajectory(obj, changeLaneCmd, currentLane, s, d, v_average)
        % Plan trajectory for the next obj.timeHorizon seconds in Frenet coordinates
            
            d_destination = 0; % Reference lane
            if currentLane == obj.lanes('ToLeftLane') || currentLane == obj.lanes('LeftLane')
                d_destination = obj.LaneWidth;
            end
            
            lengthDifference = 0;
            if changeLaneCmd
                laneChangingTrajectoryFrenet = obj.calculateLaneChangingManeuver(changeLaneCmd, s, d, v_average); 
                lengthDifference = obj.trajectoryReferenceLength - size(laneChangingTrajectoryFrenet, 1);
                if lengthDifference < 0 % Divide into current and residual lane changing trajectory
                    obj.currentTrajectoryFrenet = laneChangingTrajectoryFrenet(1:obj.trajectoryReferenceLength, :);
                    obj.residualLaneChangingTrajectory = laneChangingTrajectoryFrenet(obj.trajectoryReferenceLength+1:end, :);
                    lengthDifference = 0;
                else
                    obj.currentTrajectoryFrenet = laneChangingTrajectoryFrenet;
                end
            end
            
            s_last = obj.currentTrajectoryFrenet(end, 1);
            
            IDs_passed = s >= obj.currentTrajectoryFrenet(:, 1);
            ID_current = sum(IDs_passed);
            
            if ID_current > 1 || lengthDifference
                obj.currentTrajectoryFrenet = obj.currentTrajectoryFrenet(ID_current:end, :);
                pointsToAdd = ID_current - 1 + lengthDifference;
                durationToAdd = pointsToAdd*obj.Ts;
                
                if ~isempty(obj.residualLaneChangingTrajectory)
                    if pointsToAdd >= size(obj.residualLaneChangingTrajectory, 1)
                        addTrajectory = obj.residualLaneChangingTrajectory;
                        
                        durationToAdd = (pointsToAdd - size(obj.residualLaneChangingTrajectory, 1))*obj.Ts;
                        
                        addTrajectory = [addTrajectory; obj.calculateStraightTrajectory(s_last, d_destination, v_average, durationToAdd)];
                        obj.residualLaneChangingTrajectory = [];
                    else
                        addTrajectory = obj.residualLaneChangingTrajectory(1:pointsToAdd, :);
                        obj.residualLaneChangingTrajectory = obj.residualLaneChangingTrajectory(pointsToAdd+1:end ,:);
                    end
                else
                    addTrajectory = obj.calculateStraightTrajectory(s_last, d_destination, v_average, durationToAdd);
                end

                obj.currentTrajectoryFrenet = [obj.currentTrajectoryFrenet; addTrajectory];
            end
            
            if size(obj.currentTrajectoryFrenet, 1) ~= obj.trajectoryReferenceLength 
                error('Trajectory length is incorrect'); % For debugging
            end
            
            trajectoryFrenet = obj.currentTrajectoryFrenet;
        end
        
        function laneChangingTrajectoryFrenet = calculateLaneChangingManeuver(obj, changeLaneCmd, s, d, v_average)
        % Check whether to start or stop executing a lane changing maneuver
            
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
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current before lane change
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % deltaT: time to finish maneuver

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
            dDot_trajectory = obj.a1 + 2*obj.a2*t_discrete + 3*obj.a3*t_discrete.^2 + 4*obj.a4*t_discrete.^3 + 5*obj.a5*t_discrete.^4;
            
            laneChangingTrajectoryFrenet = [s_trajectory', d_trajectory',  dDot_trajectory'];
        end  
        
        function straightTrajectoryFrenet = calculateStraightTrajectory(obj, s_current, d_current, v_average, duartion)
        % Calculate straight trajectory staying on the same lane
        
            t_discrete = obj.Ts:obj.Ts:duartion; 
            s_trajectory = s_current + v_average*t_discrete;
            d_trajectory = d_current*ones(1, length(t_discrete));
            
            straightTrajectoryFrenet = [s_trajectory', d_trajectory',  zeros(length(t_discrete), 1)]; % TODO: dDot probably incorrect for curved road
        end
        
        function trajectoryToPlot = getTrajectoryToPlot(obj, trajectoryCartesian, currentLane)
        % Return trajectory with reduced samples for plotting  
            
            trajectoryToPlot = trajectoryCartesian;
            % TODO: Find a way to output variable size
%             if currentLane == obj.lanes('ToLeftLane') || currentLane == obj.lanes('ToRightLane')
%                 trajectoryToPlot = [trajectoryCartesian(1:obj.timeHorizon*10:end, 1:2); trajectoryCartesian(end, 1:2)]; % Reduce points to plot
%             else
%                 trajectoryToPlot = [trajectoryCartesian(1, 1:2); trajectoryCartesian(end, 1:2)]; % For straight line two points are sufficient
%             end
        end
        
        function [s_ref, d_ref, dDot_ref] = getNextTrajectoryWaypoint(obj, s)
        % Get the next waypoint for trajectory according to current s
            
            IDs_passedWPs = s >= obj.currentTrajectoryFrenet(:, 1);
            ID_nextWP = sum(IDs_passedWPs) + 1;
            
            s_ref = obj.currentTrajectoryFrenet(ID_nextWP, 1);
            d_ref = obj.currentTrajectoryFrenet(ID_nextWP, 2);
            dDot_ref = obj.currentTrajectoryFrenet(ID_nextWP, 3);
        end
    end
end

