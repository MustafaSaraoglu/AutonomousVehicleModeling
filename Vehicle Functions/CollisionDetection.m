classdef CollisionDetection < matlab.System
% Check for vehicle collisions
    
    properties(Nontunable)
        dimensionsEgo % Dimensions (length, width) ego vehicle
        wheelBaseEgo % Wheel base ego vehicle
        radiusEgo % Radius around ego vehicle rectangle representation
        dimensionsLead % Dimensions (length, width) lead vehicle
        wheelBaseLead % Wheel base lead vehicle
        radiusLead % Radius around lead vehicle rectangle representation
    end

    methods(Access = protected)
        function isCollided = stepImpl(obj, poseLead, poseEgo)
        % Check whether a collision between two vehicles is detected
            isCollided = false; 
            
            centerPointEgo = getVehicleCenterPoint(poseEgo, obj.wheelBaseEgo);
            centerPointLead = getVehicleCenterPoint(poseLead, obj.wheelBaseLead);
            
            % Only check for collision if vehicles are close 
            euclidianDistance = obj.calculateEuclidianDistance(centerPointEgo, centerPointLead);
            checkForCollision = euclidianDistance <= obj.radiusEgo + obj.radiusLead;
            if checkForCollision
                [~, ~, HitboxEgo] = createRectangleVehicle(centerPointEgo, poseEgo(3), obj.dimensionsEgo);
                [~, ~, HitboxLead] = createRectangleVehicle(centerPointLead, poseLead(3), obj.dimensionsLead);
                isCollided = obj.checkIntersection(HitboxLead, HitboxEgo);
            end
        end
        
        function out = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "boolean";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
    
    methods(Static)    
        function euclidianDistance = calculateEuclidianDistance(P1, P2)
        % Calculate Euclidian distance between two points P1(x1, y1) and P2(x2, y2)
            
            euclidianDistance = sqrt((P2(1) - P1(1))^2 + (P2(2) - P2(2))^2);
        end
        
        % FROM MOBATSim
        function CollisionFlag = checkIntersection(BoxA, BoxB)
            % Check if there is an intersection between two rectangle hitboxes
            CornersAx = transpose(BoxA(1,:));
            CornersAy = transpose(BoxA(2,:));
            CornersBx = transpose(BoxB(1,:));
            CornersBy = transpose(BoxB(2,:));
            
            in = inpolygon(CornersAx,CornersAy,CornersBx,CornersBy);
            
            if max(in) > 0
                CollisionFlag = true;
                return;
            else
                in = inpolygon(CornersBx,CornersBy,CornersAx,CornersAy);
                if max(in) > 0
                    CollisionFlag = true;
                    % To plot the collision scene
                    %plot(CornersAx,CornersAy,CornersBx,CornersBy)
                    return;
                else
                    CollisionFlag = false;
                    return;
                end 
            end
        end
    end
end
