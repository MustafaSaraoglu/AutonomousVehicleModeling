classdef DiscretePlannerFormal < DecisionMaking
% Select driving mode and decide if to execute lane changing maneuver according to formal design

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@DecisionMaking(obj); 
            
            stateNames = {...
                % Keep Lane
                'FreeDrive', 1;
                'VehicleFollowing', 2;
                'EmergencyBrake', 3;
                
                % Change Lane
                'ChangeLane', 4;
                };
            obj.states = containers.Map(stateNames(:, 1)', [stateNames{:, 2}]);
            
            % Initial state: Free Drive and on the right lane 
            obj.currentState = obj.states('FreeDrive');
            disp('@t=0s: Initial state is: ''FreeDrive''.');
        end
        
        function [changeLaneCmd, plannerMode, drivingMode] = stepImpl(obj, poseEgo, ids_surroundingVehicles, distances2surroundingVehicles, speedsOtherVehicles, vEgo)
        % Return lane change command, the current lane state and the current driving mode (see system description)
            
            [~, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]);
            
            obj.previousState = obj.currentState;
            [drivingMode, changeLaneCmd] = obj.makeDecision(dEgo, vEgo, ids_surroundingVehicles, distances2surroundingVehicles, speedsOtherVehicles);
            
            obj.displayNewState(obj.currentState, obj.previousState);
            
            plannerMode = obj.plannerModes('FORMAL');
        end
        
        function [drivingMode, changeLaneCmd] = makeDecision(obj, dEgo, vEgo, ids_surroundingVehicles, distances2surroundingVehicles, speedsOtherVehicles)
        % Make decision about driving mode and whether to change lane   
        
            % Necessary to return some output even if there is no command
            changeLaneCmd = obj.laneChangeCmds('CmdIdle');
            
            obj.currentState = obj.states(evalin('base', 'nextState'));
            
            drivingMode = obj.getStateInfo(obj.currentState);
        end
        
        function drivingMode = getStateInfo(obj, state)
        % Get information about driving mode given a state
            
            switch state
                case obj.states('FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                case obj.states('VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                case obj.states('EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                case obj.states('ChangeLane')
                    drivingMode = obj.drivingModes('FreeDrive');
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
    end
    
    methods(Static)
    end
end
