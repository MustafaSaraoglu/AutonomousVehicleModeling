classdef Decision  < matlab.System
    % Select Driving Mode and if to execute lane changing maneuver
    %
    % Driving Mode
    %   1 = FreeDrive
    %   2 = VehicleFollowing
    %   3 = EmergencyBrake
    % Change Lane
    %   0 = Command to stay in the same lane
    %   1 = Command to change to left lane
    %  -1 = Command to change to right lane
    
    properties(Nontunable)
        LaneWidth = 3.7;
    end
    
    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        currentDrivingMode 
        % Transition distances
        d_toEmergeny
        d_toFreeDrive
        d_EmergencyToFollow
        d_FreeDriveToFollow
        % Relative distance threshold to start lane changing maneuver
        s_threshold
        % Current lane state
        currentLane
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.currentDrivingMode = 1;

            obj.d_toEmergeny = 10;
            obj.d_toFreeDrive = 50;
            obj.d_EmergencyToFollow = 15;
            obj.d_FreeDriveToFollow = 39;
            
            obj.s_threshold = 40;
            
            obj.currentLane = 0; % Right lane
        end

        function [changeLane, currentLane, drivingMode] = stepImpl(obj, y_ego, relativeDistance, vLead, vEgo)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            changeLane = 0;
            
            %% Lane changing maneuver
            switch obj.currentLane
                % Right lane
                case 0
                    if vEgo > vLead && relativeDistance < obj.s_threshold && relativeDistance > 0
                        obj.currentLane = 0.5;
                        changeLane = 1;
                    end
                % To left lane
                case 0.5
                    if y_ego >= obj.LaneWidth
                        obj.currentLane = 1;
                    end
                % Left lane
                case 1
                    if vEgo > vLead && relativeDistance < -obj.s_threshold/2
                        obj.currentLane = -0.5;
                        changeLane = -1;
                    end
                % To right lane
                case -0.5
                    if y_ego <= 0
                        obj.currentLane = 0;
                    end
            end
            
            currentLane = obj.currentLane;
            
            %% Logic for switching driving mode
            
            % FreeDrive while lane changing maneuver or if leading vehicle
            % was overtaken
            if obj.currentLane ~= 0 || relativeDistance < 0
                obj.currentDrivingMode = 1;
                drivingMode = obj.currentDrivingMode;
                return
            end
            
            switch obj.currentDrivingMode
                % FreeDrive
                case 1
                    if relativeDistance <= obj.d_FreeDriveToFollow
                        if relativeDistance <= obj.d_toEmergeny
                            obj.currentDrivingMode = 3;
                        else
                            obj.currentDrivingMode = 2;
                        end
                    end
                % VehicleFollowing
                case 2
                    if relativeDistance > obj.d_toFreeDrive
                        obj.currentDrivingMode = 1;
                    elseif relativeDistance <= obj.d_toEmergeny
                        obj.currentDrivingMode = 3;
                    end
                % EmergencyBrake
                case 3
                    if relativeDistance > obj.d_EmergencyToFollow
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
