classdef Decision < matlab.System
% Select driving mode and decide if to execute lane changing maneuver
%
% Driving Mode
%   1 = FreeDrive
%   2 = VehicleFollowing
%   3 = EmergencyBrake
%   4 = LaneChanging
% Change Lane Cmd
%   0 = Command to follow current trajectory (straight/lane change)
%   1 = Command to start changing to left lane
%  -1 = Command to start changing to right lane
% Current Lane
%   0 = On the left lane
%   0.5 = Going to the right lane
%   1 = On the right lane
%  -0.5 = Going to the left lane


    properties(Nontunable)
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        Ts % Sample time
    end
    
    % Pre-computed constants
    properties(Access = private)
        drivingModes % Possible driving modes
        currentDrivingMode % Current driving mode
        
        % Transition distances for selecting driving mode
        toEmergency
        toFreeDrive
        EmergencyToFollow
        FreeDriveToFollow
        
        s_threshold % Relative distance threshold to start lane changing maneuver
        tolerance % Accepted tolerance to reach destination lane
        
        lanes % Possible lane states
        currentLane % Current lane state
        
        laneChangeCmds % Possible commands for lane changing
        
        isCompletedOvertaking % Indicate if overtaking is completed
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.drivingModes = ...
                containers.Map({'FreeDrive', 'VehicleFollowing', 'EmergencyBrake', 'LaneChanging'}, [1, 2, 3, 4]);
            obj.currentDrivingMode = obj.drivingModes('FreeDrive');

            obj.toEmergency = 10;
            obj.toFreeDrive = 50;
            obj.EmergencyToFollow = 15;
            obj.FreeDriveToFollow = 39;
            
            obj.s_threshold = 40; % Constant threshold
            obj.tolerance = 0.05;
            
            obj.lanes = ...
                containers.Map({'RightLane', 'ToLeftLane', 'LeftLane', 'ToRightLane'}, [0, 0.5, 1, -0.5]);
            obj.currentLane = obj.lanes('RightLane');
            
            obj.laneChangeCmds = ...
                containers.Map({'CmdFollow', 'CmdStartToLeft', 'CmdStartToRight'}, [0, 1, -1]);
            
            obj.isCompletedOvertaking = false;
        end
        
        function [changeLaneCmd, currentLane, drivingMode] = stepImpl(obj, poseEgo, deltaS, vLead, vEgo)
        % Return lane change command, the current lane state and the current driving mode (see system description)
        
            [~, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]);

            [changeLaneCmd, currentLane] = obj.setLaneChangingManeuver(deltaS, dEgo, vEgo, vLead);
            
            drivingMode = obj.selectDrivingMode(deltaS);
        end
        
        function [changeLaneCmd, currentLane] = setLaneChangingManeuver(obj, deltaS, dEgo, vEgo, vLead)
        % Set the command whether to start or stop a lane changing maneuver, also set the lane state accordingly
            
            % If there is no command to change lane, follow current trajectory
            changeLaneCmd = obj.laneChangeCmds('CmdFollow');
            
            switch obj.currentLane
                case obj.lanes('RightLane')
                    if vEgo > vLead && deltaS < obj.s_threshold && deltaS > 0
                        changeLaneCmd = obj.laneChangeCmds('CmdStartToLeft');
                        obj.currentLane = obj.lanes('ToLeftLane');
                    end
                case obj.lanes('ToLeftLane')
                    if abs(obj.LaneWidth - dEgo) < obj.tolerance % Reached left lane
                        obj.currentLane = obj.lanes('LeftLane');
                    end
                case obj.lanes('LeftLane')
                    if vEgo > vLead && deltaS < -obj.s_threshold/2
                        changeLaneCmd = obj.laneChangeCmds('CmdStartToRight');
                        obj.currentLane = obj.lanes('ToRightLane');
                    end
                case obj.lanes('ToRightLane')
                    if abs(0 - dEgo) < obj.tolerance % Reached right lane
                        obj.currentLane = obj.lanes('RightLane');
                        obj.isCompletedOvertaking = true;
                    end
            end
            
            currentLane = obj.currentLane;
        end
        
        function drivingMode = selectDrivingMode(obj, deltaS)
        % Select driving mode according to transition conditions
            
            if obj.isCompletedOvertaking
                obj.currentDrivingMode = obj.drivingModes('FreeDrive');
                drivingMode = obj.currentDrivingMode;
                return
            end
        
            if obj.currentLane ~= obj.lanes('RightLane')
                obj.currentDrivingMode = obj.drivingModes('LaneChanging');
                drivingMode = obj.currentDrivingMode;
                return
            end
            
            switch obj.currentDrivingMode
                case obj.drivingModes('FreeDrive')
                    if deltaS <= obj.FreeDriveToFollow
                        if deltaS <= obj.toEmergency
                            obj.currentDrivingMode = obj.drivingModes('EmergencyBrake');
                        else
                            obj.currentDrivingMode = obj.drivingModes('VehicleFollowing');
                        end
                    end
                case obj.drivingModes('VehicleFollowing')
                    if deltaS > obj.toFreeDrive
                        obj.currentDrivingMode = obj.drivingModes('FreeDrive');
                    elseif deltaS <= obj.toEmergency
                        obj.currentDrivingMode = obj.drivingModes('EmergencyBrake');
                    end
                case obj.drivingModes('EmergencyBrake')
                    if deltaS > obj.EmergencyToFollow
                        obj.currentDrivingMode = obj.drivingModes('VehicleFollowing');
                    end   
            end
            drivingMode = obj.currentDrivingMode;
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];
            out3 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end 
        
%         function sts = getSampleTimeImpl(obj)
%             % Define sample time type and parameters
% 
%             % Example: specify discrete sample time
%             sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.Ts);
%         end      
    end
end
