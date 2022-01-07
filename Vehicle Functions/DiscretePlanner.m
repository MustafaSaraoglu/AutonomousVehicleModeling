classdef DiscretePlanner < matlab.System
% Select driving mode and decide if to execute lane changing maneuver

% Driving Mode
%   1 = FreeDrive
%   2 = VehicleFollowing
%   3 = EmergencyBrake

% Current Lane
%   0 = On the right lane
%   0.5 = Going to left lane
%   1 = On the left lane
%   -0.5 = Going to right lane

% Change Lane Cmd
%   0 = Command to follow current trajectory (left lane, right lane, lane change)
%   1 = Command to start changing to left lane
%  -1 = Command to start changing to right lane

    properties(Nontunable)
        vEgo_ref % Reference velocity for ego vehicle [m/s]
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        Ts % Sample time [s]
    end
    
    % Pre-computed constants
    properties(Access = private)
        drivingModes % Possible driving modes
        lanes % Possible lane states
        laneChangeCmds % Possible commands for lane changing
        
        states % Possible driving states
        currentState % Current driving state
        
        toleranceReachLane % Accepted tolerance to reach destination lane
        
        waitingCounter % Counter to wait some time before recheck for lane changing
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.drivingModes = ...
                containers.Map({'FreeDrive', 'VehicleFollowing', 'EmergencyBrake'}, [1, 2, 3]);
            
            obj.lanes = ...
                containers.Map({'RightLane', 'ToLeftLane', 'LeftLane', 'ToRightLane'}, [0, 0.5, 1, -0.5]);
            
            obj.laneChangeCmds = ...
                containers.Map({'CmdIdle', 'CmdStartToLeftLane', 'CmdStartToRightLane'}, [0, 1, -1]);
            
            stateNames = {...
                % Right lane
                'RightLane_FreeDrive', 1;
                'RightLane_VehicleFollowing', 2;
                'RightLane_EmergencyBrake', 3;
                
                % To left lane
                'ToLeftLane_FreeDrive', 4;
                'ToLeftLane_VehicleFollowing', 5;
                'ToLeftLane_EmergencyBrake', 6;
                
                % Left lane
                'LeftLane_FreeDrive', 7;
                'LeftLane_VehicleFollowing', 8;
                'LeftLane_EmergencyBrake', 9;
                
                % To right lane
                'ToRightLane_FreeDrive', 10;
                'ToRightLane_VehicleFollowing', 11;
                'ToRightLane_EmergencyBrake', 12;
                
                % Check to other lane
                'CheckToLeftLane_VehicleFollowing', 13;
                'CheckToRightLane_FreeDrive', 14;
                'CheckToRightLane_VehicleFollowing', 15
                };
            obj.states = containers.Map(stateNames(:, 1)', [stateNames{:, 2}]);
            
            obj.currentState = obj.states('RightLane_FreeDrive');
            
            obj.toleranceReachLane = 0.05;
            
            obj.waitingCounter = 0;
        end
        
        function [changeLaneCmd, currentLane, drivingMode] = stepImpl(obj, poseEgo, delta_s_surroundingVehicles, v_surroundingVehicles, vEgo)
        % Return lane change command, the current lane state and the current driving mode (see system description)
            
            [~, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]);
            vLead = v_surroundingVehicles(1);
           
            [drivingMode, currentLane, changeLaneCmd] = obj.makeDecision(dEgo, vEgo, delta_s_surroundingVehicles, vLead);
        end
        
        function [drivingMode, currentLane, changeLaneCmd] = makeDecision(obj, dEgo, vEgo, delta_s_surroundingVehicles, vLead)
        % Make decision about driving mode and whether to change lane    
            
            if obj.waitingCounter > 0
                obj.waitingCounter = obj.waitingCounter - 1;
            end
        
            delta_s_frontSameLane = delta_s_surroundingVehicles(1);
            delta_s_rearSameLane = delta_s_surroundingVehicles(2);
            delta_s_frontOtherLane = delta_s_surroundingVehicles(3);
            delta_s_rearOtherLane = delta_s_surroundingVehicles(4);
        
            changeLaneCmd = obj.laneChangeCmds('CmdIdle');
        
            switch obj.currentState % TODO: Change to elseif after checking only one statement can be true
                case obj.states('RightLane_FreeDrive')
                    if obj.isVehicleInFrontClose(delta_s_frontSameLane) && not(obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)) 
                        obj.currentState = obj.states('RightLane_VehicleFollowing');
                    end
                    
                    if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)
                        obj.currentState = obj.states('RightLane_EmergencyBrake');
                    end
                    
                case obj.states('RightLane_VehicleFollowing')
                    if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane) 
                        obj.currentState = obj.states('RightLane_EmergencyBrake');
                    end
                    
                    if obj.isVehicleInFrontVeryFar(delta_s_frontSameLane)
                        obj.currentState = obj.states('RightLane_FreeDrive');
                    end
                    
                    if obj.isVehicleInFrontClose(delta_s_frontSameLane) && not(obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)) && ...
                            obj.isVehicleInFrontSlower(vEgo, vLead) && not(obj.isCloseToReferenceSpeed(vEgo, obj.vEgo_ref)) && ...
                            not(obj.isOtherLaneOccupied(delta_s_frontOtherLane, delta_s_rearOtherLane)) && ...
                            obj.isWaitingTimeOver(obj.waitingCounter)
                        obj.currentState = obj.states('CheckToLeftLane_VehicleFollowing');
                        
                        changeLaneCmd = obj.laneChangeCmds('CmdStartToLeftLane');
                    end
                    
                case obj.states('RightLane_EmergencyBrake')
                    if obj.isVehicleInFrontFar(delta_s_frontSameLane)
                        obj.currentState = obj.states('RightLane_VehicleFollowing');
                    end
                    
                case obj.states('CheckToLeftLane_VehicleFollowing')
                    if obj.isTrajectoryAccepted()
                        obj.currentState = obj.states('ToLeftLane_FreeDrive');
                    end
                    
                    if not(obj.isTrajectoryAccepted())
                        obj.currentState = obj.states('RightLane_VehicleFollowing');
                        obj.waitingCounter = obj.setWaitingCounter(); 
                    end
                    
                case obj.states('ToLeftLane_FreeDrive')
%                     if obj.isVehicleInFrontClose(delta_s_frontSameLane) && not(obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)) && ...
%                             not(obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToLeftLane_VehicleFollowing');
%                     end
%                     
%                     if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane) && ...
%                             not(obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToLeftLane_EmergencyBrake');
%                     end
                    
                    if obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane)
                        obj.currentState = obj.states('LeftLane_FreeDrive');
                    end
                        
%                 case obj.states('ToLeftLane_VehicleFollowing')
%                     if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane) && ...
%                             not(obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane)) 
%                         obj.currentState = obj.states('ToLeftLane_EmergencyBrake');
%                     end
%                     
%                     if obj.isVehicleInFrontVeryFar(delta_s_frontSameLane) && ...
%                             not(obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToLeftLane_FreeDrive');
%                     end
%                     
%                     if obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane)
%                         obj.currentState = obj.states('LeftLane_VehicleFollowing');
%                     end
                    
%                 case obj.states('ToLeftLane_EmergencyBrake')
%                     if obj.isVehicleInFrontFar(delta_s_frontSameLane) && ...
%                             not(obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToLeftLane_VehicleFollowing');
%                     end
%                     
%                     if obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane)
%                         obj.currentState = obj.states('LeftLane_EmergencyBrake');
%                     end
                    
                case obj.states('LeftLane_FreeDrive')
                    if obj.isVehicleInFrontClose(delta_s_frontSameLane) && not(obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)) && obj.isOtherLaneOccupied(delta_s_frontOtherLane, delta_s_rearOtherLane)
                        obj.currentState = obj.states('LeftLane_VehicleFollowing');
                    end
                    
                    if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)
                        obj.currentState = obj.states('LeftLane_EmergencyBrake');
                    end
                    
                    if not(obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)) && not(obj.isOtherLaneOccupied(delta_s_frontOtherLane, delta_s_rearOtherLane)) && ...
                            obj.isWaitingTimeOver(obj.waitingCounter)
                        obj.currentState = obj.states('CheckToRightLane_FreeDrive');
                        
                        changeLaneCmd = obj.laneChangeCmds('CmdStartToRightLane');
                    end
                    
                case obj.states('LeftLane_VehicleFollowing')
                    if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)
                        obj.currentState = obj.states('LeftLane_EmergencyBrake');
                    end
                    
                    if obj.isVehicleInFrontVeryFar(delta_s_frontSameLane) && obj.isOtherLaneOccupied(delta_s_frontOtherLane, delta_s_rearOtherLane)
                        obj.currentState = obj.states('LeftLane_FreeDrive');
                    end
                    
                    if not(obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)) && not(obj.isOtherLaneOccupied(delta_s_frontOtherLane, delta_s_rearOtherLane)) && ...
                            obj.isWaitingTimeOver(obj.waitingCounter)
                        obj.currentState = obj.states('CheckToRightLane_VehicleFollowing');
                        
                        changeLaneCmd = obj.laneChangeCmds('CmdStartToRightLane');
                    end
                    
                case obj.states('LeftLane_EmergencyBrake')
                    if obj.isVehicleInFrontFar(delta_s_frontSameLane)
                        obj.currentState = obj.states('LeftLane_VehicleFollowing');
                    end
                
                case obj.states('CheckToRightLane_FreeDrive')
                    if obj.isTrajectoryAccepted()
                        obj.currentState = obj.states('ToRightLane_FreeDrive');
                    end
                    
                    if not(obj.isTrajectoryAccepted())
                        obj.currentState = obj.states('LeftLane_FreeDrive');
                        obj.waitingCounter = obj.setWaitingCounter(); 
                    end
                    
                case obj.states('CheckToRightLane_VehicleFollowing')
                    if obj.isTrajectoryAccepted()
                        obj.currentState = obj.states('ToRightLane_FreeDrive');
                    end
                    
                    if not(obj.isTrajectoryAccepted())
                        obj.currentState = obj.states('LeftLane_VehicleFollowing');
                        obj.waitingCounter = obj.setWaitingCounter();
                    end
                    
                case obj.states('ToRightLane_FreeDrive')
%                     if obj.isVehicleInFrontClose(delta_s_frontSameLane) && not(obj.isVehicleInFrontVeryClose(delta_s_frontSameLane)) && ...
%                             not(obj.isReachedRightLane(dEgo, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToRightLane_VehicleFollowing');
%                     end
%                     
%                     if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane) && ...
%                             not(obj.isReachedRightLane(dEgo, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToRightLane_EmergencyBrake');
%                     end
                    
                    if obj.isReachedRightLane(dEgo, obj.toleranceReachLane)
                        obj.currentState = obj.states('RightLane_FreeDrive');
                    end
                    
%                 case obj.states('ToRightLane_VehicleFollowing')
%                     if obj.isVehicleInFrontVeryClose(delta_s_frontSameLane) && ...
%                             not(obj.isReachedRightLane(dEgo, obj.toleranceReachLane)) 
%                         obj.currentState = obj.states('ToRightLane_EmergencyBrake');
%                     end
%                     
%                     if obj.isVehicleInFrontVeryFar(delta_s_frontSameLane) && ...
%                             not(obj.isReachedRightLane(dEgo, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToRightLane_FreeDrive');
%                     end
%                     
%                     if obj.isReachedRightLane(dEgo, obj.toleranceReachLane)
%                         obj.currentState = obj.states('RightLane_VehicleFollowing');
%                     end

%                 case obj.states('ToRightLane_EmergencyBrake')
%                     if obj.isVehicleInFrontFar(delta_s_frontSameLane) && ...
%                             not(obj.isReachedRightLane(dEgo, obj.toleranceReachLane))
%                         obj.currentState = obj.states('ToRightLane_VehicleFollowing');
%                     end
%                     
%                     if obj.isReachedRightLane(dEgo, obj.toleranceReachLane)
%                         obj.currentState = obj.states('RightLane_EmergencyBrake');
%                     end
            end
            
            [drivingMode, currentLane] = obj.getStateInfo(obj.currentState);
        end
        
        function [drivingMode, currentLane] = getStateInfo(obj, state)
        % Get information about drivingMode and currentLane given a state
            
            switch state
                case obj.states('RightLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                    currentLane = obj.lanes('RightLane');
                case obj.states('RightLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                    currentLane = obj.lanes('RightLane');
                case obj.states('RightLane_EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                    currentLane = obj.lanes('RightLane');
                case obj.states('ToLeftLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                    currentLane = obj.lanes('ToLeftLane');
                case obj.states('ToLeftLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                    currentLane = obj.lanes('ToLeftLane');
                case obj.states('ToLeftLane_EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                    currentLane = obj.lanes('ToLeftLane');
                case obj.states('LeftLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                    currentLane = obj.lanes('LeftLane');
                case obj.states('LeftLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                    currentLane = obj.lanes('LeftLane');
                case obj.states('LeftLane_EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                    currentLane = obj.lanes('LeftLane');
                case obj.states('ToRightLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                    currentLane = obj.lanes('ToRightLane');
                case obj.states('ToRightLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                    currentLane = obj.lanes('ToRightLane');
                case obj.states('ToRightLane_EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                    currentLane = obj.lanes('ToRightLane');
                case obj.states('CheckToLeftLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                    currentLane = obj.lanes('RightLane');
                case obj.states('CheckToRightLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                    currentLane = obj.lanes('LeftLane');
                case obj.states('CheckToRightLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                    currentLane = obj.lanes('LeftLane');
            end
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
    
    methods(Static)
        % TODO: Decide what is vehicle in front
        % TODO: Close in dynamic sense not fixed constant
        function isClose = isVehicleInFrontClose(distanceToFrontVehicle)
            isClose = distanceToFrontVehicle <= 40;
        end
        
        function isVeryClose = isVehicleInFrontVeryClose(distanceToFrontVehicle)
            isVeryClose = distanceToFrontVehicle <= 10;
        end
        
        function isFar = isVehicleInFrontFar(distanceToFrontVehicle)
            isFar = distanceToFrontVehicle >= 15;
        end
        
        function isVeryFar = isVehicleInFrontVeryFar(distanceToFrontVehicle)
            isVeryFar = distanceToFrontVehicle >= 50;
        end
        
        
        function isSlower = isVehicleInFrontSlower(vEgo, vLead)
            isSlower = vEgo - vLead >= -0.1; % Some tolerance
        end
        
        function isSlower = isVehicle2OverTakeSlower(vEgo, v2Overtake)
            isSlower = v2Overtake < vEgo;
        end
        
        function isClose = isCloseToReferenceSpeed(vEgo, vEgo_ref)
            isClose = vEgo >= 0.95*vEgo_ref;
        end
        
        % TODO: Find correct conditions
        function isOccupied = isOtherLaneOccupied(distanceToFrontVehicleOtherLane, distanceToRearVehicleOtherLane)
            isOccupied = distanceToFrontVehicleOtherLane < 50 || distanceToRearVehicleOtherLane > -50;
        end
        
        function isReached = isReachedLeftLane(dEgo, laneWidth, tolerance)
            isReached = (abs(laneWidth - dEgo) < tolerance);
        end
        
        function isReached = isReachedRightLane(dEgo, tolerance)
            isReached = (abs(0 - dEgo) < tolerance);
        end
        
        
        function isOver = isWaitingTimeOver(counter)
            isOver = counter == 0;
        end
        
        function counter = setWaitingCounter()
            counter = 1000; % Set counter to wait for next lane change command
        end
        
        function isAccepted = isTrajectoryAccepted()
            % TODO: Implementation in Simulink without causing algebraic
            % loop or messing up execution order (witout using base workespace)
            isAccepted = evalin('base', 'isAcceptedTrajectory');
        end
    end
end
