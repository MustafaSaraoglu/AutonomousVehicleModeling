classdef CollisionDetection < matlab.System
% Check for vehicle collisions
    
    properties(Nontunable)
        dimensionsEgo % Dimensions (length, width) ego vehicle
        wheelBaseEgo % Wheel base ego vehicle
        
        dimensionsOtherVehicles % Dimensions (length, width) other vehicles
        wheelBaseOtherVehicles % Wheel base other vehicles   
    end
    
    properties(Access = protected)
        radiusEgo % Radius around ego vehicle rectangle representation
        radiusOtherVehicles % Radius around other vehicles rectangle representation
        isCollisionDetected % Boolean to only register first collision during simulation
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.radiusEgo = sqrt((obj.dimensionsEgo(1)/2)^2 + (obj.dimensionsEgo(2)/2)^2); 
            obj.radiusOtherVehicles = sqrt((obj.dimensionsOtherVehicles(1, :)/2).^2 + ...
                (obj.dimensionsOtherVehicles(2, :)/2).^2);
            obj.isCollisionDetected = false;
        end
        
        function isCollided = stepImpl(obj, poseOtherVehicles, poseEgo)
        % Check whether a collision between two vehicles is detected
            
            isCollided = false; 
            
            % Center of rectangle vehicle is needed to check if circle
            % around vehicles are overlapping
            centerPointEgo = getVehicleCenterPoint(poseEgo, obj.wheelBaseEgo);
            centerPointOtherVehicles = getVehicleCenterPoint(poseOtherVehicles, ...
                                                             obj.wheelBaseOtherVehicles);
            
            % Only check for collision if vehicles are close 
            euclidianDistance = obj.calculateEuclidianDistance(centerPointEgo, ...
                                                               centerPointOtherVehicles);
            checkForCollision = euclidianDistance <= obj.radiusEgo + obj.radiusOtherVehicles;
            if any(checkForCollision)
                [~, ~, HitboxEgo] = createRectangleVehicle(centerPointEgo, poseEgo(3), ...
                                                           obj.dimensionsEgo);
                
                for id_otherVehicle = find(checkForCollision)
                    [~, ~, HitboxOtherVehicle] = ...
                        createRectangleVehicle(centerPointOtherVehicles(:, id_otherVehicle), ...
                                               poseOtherVehicles(3, id_otherVehicle), ...
                                               obj.dimensionsOtherVehicles(:, id_otherVehicle));
                    
                    % Check if rectangles are overlapping
                    isCollided = obj.checkIntersection(HitboxOtherVehicle, HitboxEgo);
                    
                    if isCollided
                        if ~obj.isCollisionDetected
                            t = get_param('ManeuverPlanning', 'SimulationTime');

                            %fprintf('@t=%fs: Collision with other vehicle %i.\n', t, id_otherVehicle);
                            obj.isCollisionDetected = true;
                        end
                        return
                    end
                end
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

            euclidianDistance = sqrt((P2(1, :) - P1(1, :)).^2 + (P2(2, :) - P2(2, :)).^2);
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
