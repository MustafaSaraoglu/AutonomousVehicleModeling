classdef Decision  < CoordinateTransformations
    % Select driving mode and decide if to execute lane changing maneuver
    %
    % Driving Mode
    %   1 = FreeDrive
    %   2 = VehicleFollowing
    %   3 = EmergencyBrake
    % Change Lane
    %   0 = Command to follow current trajectory (straight/lane change)
    %   1 = Command to start changing to left lane
    %  -1 = Command to start changing to right lane
    %   2 = Command to stop lane changing maneuver
    
    properties(Nontunable)
        
    end
    
    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        currentDrivingMode % Driving mode
        
        % Transition distances
        toEmergeny
        toFreeDrive
        EmergencyToFollow
        FreeDriveToFollow
        
        s_threshold % Relative distance threshold to start lane changing maneuver
        
        currentLane % Current lane state
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.currentDrivingMode = 1;

            obj.toEmergeny = 10;
            obj.toFreeDrive = 50;
            obj.EmergencyToFollow = 15;
            obj.FreeDriveToFollow = 39;
            
            obj.s_threshold = 40; % Constant threshold
            
            obj.currentLane = 0; % Right lane
        end

        function [changeLane, currentLane, drivingMode] = stepImpl(obj, poseEgo, deltaS, vLead, vEgo)
            % Return command whether to change lane, the current lane state 
            % (left, right, left to right, right to left) the and the 
            % current driving mode (see system description)

            % Cartesian to Frenet coordinate transformation
            [~, dEgo] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [poseEgo(1) poseEgo(2)]); % Determine current <s,d>

            % By default follow current trajectory
            changeLane = 0;
            
            %% Lane changing maneuver
            switch obj.currentLane
                % Right lane
                case 0
                    if vEgo > vLead && deltaS < obj.s_threshold && deltaS > 0
                        obj.currentLane = 0.5;
                        changeLane = 1;
                    end
                % To left lane
                case 0.5
                    if dEgo >= obj.LaneWidth
                        obj.currentLane = 1;
                        changeLane = 2;
                    end
                % Left lane
                case 1
                    if vEgo > vLead && deltaS < -obj.s_threshold/2
                        obj.currentLane = -0.5;
                        changeLane = -1;
                    end
                % To right lane
                case -0.5
                    if dEgo <= 0
                        obj.currentLane = 0;
                        changeLane = 2;
                    end
            end
            
            currentLane = obj.currentLane;
            
            %% Logic for switching driving mode
            
            % FreeDrive while lane changing maneuver or if leading vehicle
            % was overtaken
            if obj.currentLane ~= 0 || deltaS < 0
                obj.currentDrivingMode = 1;
                drivingMode = obj.currentDrivingMode;
                return
            end
            
            switch obj.currentDrivingMode
                % FreeDrive
                case 1
                    if deltaS <= obj.FreeDriveToFollow
                        if deltaS <= obj.toEmergeny
                            obj.currentDrivingMode = 3;
                        else
                            obj.currentDrivingMode = 2;
                        end
                    end
                % VehicleFollowing
                case 2
                    if deltaS > obj.toFreeDrive
                        obj.currentDrivingMode = 1;
                    elseif deltaS <= obj.toEmergeny
                        obj.currentDrivingMode = 3;
                    end
                % EmergencyBrake
                case 3
                    if deltaS > obj.EmergencyToFollow
                        obj.currentDrivingMode = 2;
                    end   
            end
            
            drivingMode = obj.currentDrivingMode;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];
            out3 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
