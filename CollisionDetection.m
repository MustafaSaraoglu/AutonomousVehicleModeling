classdef CollisionDetection < matlab.System
    % Check for vehicle collisions

    % Public, tunable properties
    properties

    end
    
    properties(Nontunable)
        rEgo = evalin('base', 'R2');
        rLead = evalin('base', 'R1');
        dimEgo = evalin('base', 'V2_dim');
        dimLead = evalin('base', 'V1_dim');
    end
    
    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function collisionDetected = stepImpl(obj, poseLead, poseEgo)
            % Implement algorithm.           
            collisionDetected = false; 
            
            % Check if circles around vehicles intersect
            intersectionPoints = circcirc(poseEgo(1), poseEgo(2), obj.rEgo, poseLead(1), poseLead(2), obj.rLead);
            
            % Only heck for collison if circles intersect
            if ~isnan(intersectionPoints(1))
                HitboxEgo = obj.getHitbox(poseEgo, obj.dimEgo);
                HitboxLead = obj.getHitbox(poseLead, obj.dimLead);
                collisionDetected = obj.checkIntersection(HitboxLead, HitboxEgo);
            end
        end
        
        function Hitbox = getHitbox(~, pose, dim)
            % Vehicle representation as rectangle
            % Vehicle pose
            x = pose(1);
            y = pose(2);
            yaw = pose(3);

            centerP = [x; y];

            V_Length = dim(1);
            V_Width = dim(2);

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
        
        %% FROM MOBATSim
        function CollisionFlag = checkIntersection(~, BoxA, BoxB)
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
        
        %%
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "boolean";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
