classdef DiscretePlannerManual < DecisionMaking
% Select driving mode and decide if to execute lane changing maneuver according to manual design

    % Pre-computed constants
    properties(Access = protected)
        T_LC % Constant duration for lane change
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@DecisionMaking(obj);
            
            obj.T_LC = 4;
            
            stateNames = {...
                % Right lane
                'RightLane_FreeDrive', 1;
                'RightLane_VehicleFollowing', 2;
                'RightLane_EmergencyBrake', 3;
                
                % To left lane
                'ToLeftLane_FreeDrive', 4;
                
                % Left lane
                'LeftLane_FreeDrive', 5;
                'LeftLane_VehicleFollowing', 6;
                'LeftLane_EmergencyBrake', 7;
                
                % To right lane
                'ToRightLane_FreeDrive', 8;
                };
            obj.states = containers.Map(stateNames(:, 1)', [stateNames{:, 2}]);
            
            % Initial state: Free Drive and on the right lane 
            obj.currentState = obj.states('RightLane_FreeDrive');
            %disp('@t=0s: Initial state is: ''RightLane_FreeDrive''.');
        end
        
        function [changeLaneCmd, drivingMode] = stepImpl(obj, poseEgo, ids_surroundingVehicles, ...
                                                         distances2surroundingVehicles, ...
                                                         speedsOtherVehicles, vEgo)
        % Return lane change command and the current driving mode 
            
            [~, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]);
            
            obj.previousState = obj.currentState;
            [drivingMode, changeLaneCmd] = obj.makeDecision(dEgo, vEgo, ids_surroundingVehicles, ...
                                                            distances2surroundingVehicles, ...
                                                            speedsOtherVehicles);
            
            obj.displayNewState(obj.currentState, obj.previousState);
        end
        
        function [drivingMode, changeLaneCmd] = makeDecision(obj, dEgo, vEgo, ...
                                                             ids_surroundingVehicles, ...
                                                             distances2surroundingVehicles, ...
                                                             speedsOtherVehicles)
        % Make decision about driving mode and whether to change lane    
            
            vLead = []; % No detection
            id_lead = ids_surroundingVehicles(1);
            if id_lead ~= -1 % Vehicle detected
                vLead = speedsOtherVehicles(id_lead);
            end
            
            % Distances to surrounding vehicles 
            distance2frontSameLane = distances2surroundingVehicles(1);
            distance2frontOtherLane = distances2surroundingVehicles(3);
            distance2rearOtherLane = distances2surroundingVehicles(4);
            
            % Necessary to return some output even if there is no command
            changeLaneCmd = obj.laneChangeCmds('CmdIdle');
            
            % Possible states: Check draw.io
            switch obj.currentState 
                case obj.states('RightLane_FreeDrive')
                    if obj.isVehicleClose(distance2frontSameLane) && ...
                            not(obj.isVehicleInFrontVeryClose(distance2frontSameLane)) 
                        obj.currentState = obj.states('RightLane_VehicleFollowing');
                    end
                    
                    if obj.isVehicleInFrontVeryClose(distance2frontSameLane)
                        obj.currentState = obj.states('RightLane_EmergencyBrake');
                    end
                    
                case obj.states('RightLane_VehicleFollowing')
                    if obj.isVehicleInFrontVeryClose(distance2frontSameLane) 
                        obj.currentState = obj.states('RightLane_EmergencyBrake');
                    end
                    
                    if obj.isVehicleIVeryFar(distance2frontSameLane)
                        
                        obj.currentState = obj.states('RightLane_FreeDrive');
                    end
                    
                    if obj.isVehicleClose(distance2frontSameLane) && ...
                            not(obj.isVehicleInFrontVeryClose(distance2frontSameLane)) && ...
                            obj.isVehicleSlower(vEgo, vLead) && ...
                            not(obj.isCloseToReferenceSpeed(vEgo, obj.vEgo_ref)) && ...
                            not(obj.isOtherLaneOccupied(distance2frontOtherLane, ...
                                                        distance2rearOtherLane))
                        obj.currentState = obj.states('ToLeftLane_FreeDrive');
                        
                        changeLaneCmd = obj.T_LC; % To left lane
                    end
                    
                case obj.states('RightLane_EmergencyBrake')
                    if obj.isVehicleFar(distance2frontSameLane)
                        obj.currentState = obj.states('RightLane_VehicleFollowing');
                    end
                    
                case obj.states('ToLeftLane_FreeDrive')   
                    if obj.isReachedLeftLane(dEgo, obj.LaneWidth, obj.toleranceReachLane)
                        obj.currentState = obj.states('LeftLane_FreeDrive');
                    end

                case obj.states('LeftLane_FreeDrive')
                    if obj.isVehicleClose(distance2frontSameLane) && ...
                            not(obj.isVehicleInFrontVeryClose(distance2frontSameLane)) && ...
                            obj.isOtherLaneOccupied(distance2frontOtherLane, distance2rearOtherLane)
                        obj.currentState = obj.states('LeftLane_VehicleFollowing');
                    end
                    
                    if obj.isVehicleInFrontVeryClose(distance2frontSameLane)
                        obj.currentState = obj.states('LeftLane_EmergencyBrake');
                    end
                    
                    if not(obj.isVehicleInFrontVeryClose(distance2frontSameLane)) && ...
                            not(obj.isOtherLaneOccupied(distance2frontOtherLane, ...
                                                        distance2rearOtherLane))
                        obj.currentState = obj.states('ToRightLane_FreeDrive');
                        
                        changeLaneCmd = obj.T_LC; % To right lane
                    end
                    
                case obj.states('LeftLane_VehicleFollowing')
                    if obj.isVehicleInFrontVeryClose(distance2frontSameLane)
                        obj.currentState = obj.states('LeftLane_EmergencyBrake');
                    end
                    
                    if obj.isVehicleIVeryFar(distance2frontSameLane) && ...
                            obj.isOtherLaneOccupied(distance2frontOtherLane, distance2rearOtherLane)
                        obj.currentState = obj.states('LeftLane_FreeDrive');
                    end
                    
                    if not(obj.isVehicleInFrontVeryClose(distance2frontSameLane)) && ...
                            not(obj.isOtherLaneOccupied(distance2frontOtherLane, ...
                                                        distance2rearOtherLane))
                        obj.currentState = obj.states('ToRightLane_FreeDrive');
                        
                        changeLaneCmd = obj.T_LC; % To right lane
                    end
                    
                case obj.states('LeftLane_EmergencyBrake')
                    if obj.isVehicleFar(distance2frontSameLane)
                        obj.currentState = obj.states('LeftLane_VehicleFollowing');
                    end
                    
                case obj.states('ToRightLane_FreeDrive')
                    if obj.isReachedRightLane(dEgo, obj.toleranceReachLane)
                        obj.currentState = obj.states('RightLane_FreeDrive');
                    end
            end
            
            drivingMode = obj.getStateInfo(obj.currentState);
        end
        
        function drivingMode = getStateInfo(obj, state)
        % Get information about driving mode given a state
            
            switch state
                case obj.states('RightLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                case obj.states('RightLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                case obj.states('RightLane_EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                case obj.states('ToLeftLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                case obj.states('LeftLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                case obj.states('LeftLane_VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                case obj.states('LeftLane_EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                case obj.states('ToRightLane_FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
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
    
    methods(Static)
        % TODO: Do not use fixed constant but dynamic values instead
        function isClose = isVehicleClose(distanceToFrontVehicle)
            isClose = distanceToFrontVehicle <= 40;
        end
        
        function isVeryClose = isVehicleInFrontVeryClose(distanceToFrontVehicle)
            isVeryClose = distanceToFrontVehicle <= 10;
        end
        
        function isFar = isVehicleFar(distanceToOtherVehicle)
            isFar = distanceToOtherVehicle >= 15;
        end
        
        function isVeryFar = isVehicleIVeryFar(distanceToOtherVehicle)
            isVeryFar = distanceToOtherVehicle >= 50;
        end
        
        
        function isSlower = isVehicleSlower(v, v_other)
            % No detection
            if isempty(v_other) 
                isSlower = true;
                return
            end
            
            isSlower = v - v_other >= -0.1; % Some tolerance
        end
        
        function isClose = isCloseToReferenceSpeed(v, v_ref)
            isClose = v >= 0.95*v_ref;
        end
        
        function isOccupied = isOtherLaneOccupied(distanceToFrontVehicleOtherLane, ...
                                                  distanceToRearVehicleOtherLane)
            isOccupied = distanceToFrontVehicleOtherLane < 50 || ...
                         distanceToRearVehicleOtherLane > -50;
        end
        
        function isReached = isReachedLeftLane(d, laneWidth, tolerance)
            isReached = (abs(laneWidth - d) < tolerance);
        end
        
        function isReached = isReachedRightLane(d, tolerance)
            isReached = (abs(0 - d) < tolerance);
        end
    end
end
