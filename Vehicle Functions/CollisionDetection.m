classdef CollisionDetection < matlab.System
    % Check for vehicle collisions
    
    properties(Nontunable)
        radiusEgo % Radius around ego vehicle rectangle representation
        radiusLead % Radius around lead vehicle rectangle representation
        dimensionsEgo % Dimensions (length, width) ego vehicle
        dimensionsLead % Dimensions (length, width) lead vehicle
    end
    
    methods(Static)    
        function euclidianDistance = calculateEuclidianDistance(x1, y1, x2, y2)
            % Calculate Euclidian distance between two points P1(x1, y1)
            % and P2(x2, y2)
            euclidianDistance = sqrt((x2 - x1)^2 + (y2 - y1)^2);
        end
        
        function Hitbox = getHitbox(pose, dimension)
            % Get vehicle hitbox (representation as rectangle)
            
            % Vehicle pose
            x = pose(1);
            y = pose(2);
            yaw = pose(3);

            centerP = [x; y];

            V_Length = dimension(1);
            V_Width = dimension(2);

            % Vehicle as rectangle
            p1 = [V_Length/2; V_Width/2];
            p2 = [V_Length/2; -V_Width/2];
            p3 = [-V_Length/2; -V_Width/2];
            p4 = [-V_Length/2; V_Width/2];

            % Rotation of rectangle points
            Rmatrix = [cos(yaw) -sin(yaw);
                       sin(yaw)  cos(yaw)];

            p1r = centerP + Rmatrix*p1;
            p2r = centerP + Rmatrix*p2;
            p3r = centerP + Rmatrix*p3;
            p4r = centerP + Rmatrix*p4;

            % Connect points to rectangle
            Hitbox = [p1r p2r p3r p4r];
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

    methods(Access = protected)
        function collisionDetected = stepImpl(obj, poseLead, poseEgo)
            % Check whether a collision between two vehicles is detected
            
            collisionDetected = false; 
            
            % Only check if vehicles are close using euclidian distance to
            % reduce computational resources
            euclidianDistance = obj.calculateEuclidianDistance(poseEgo(1), poseEgo(2), poseLead(1), poseLead(2));
            if euclidianDistance <= obj.radiusEgo + obj.radiusLead
                HitboxEgo = obj.getHitbox(poseEgo, obj.dimensionsEgo);
                HitboxLead = obj.getHitbox(poseLead, obj.dimensionsLead);
                collisionDetected = obj.checkIntersection(HitboxLead, HitboxEgo);
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
end
