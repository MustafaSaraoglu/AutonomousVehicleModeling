classdef LocalTrajectoryPlanner < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
    %LocalTrajectoryPlanner Superclass for generating necessary inputs for the lateral controllers.
    %   Detailed explanation goes here
    
    properties
        
    end
    
    properties(Nontunable)
        LaneWidth = evalin('base', 'w_lane');
    end
    
    % Pre-computed constants
    properties(Access = protected)
        % Coefficents for lane changing trajectory
        a0
        a1
        a2
        a3
        a4
        a5
        % State if any maneuver should be executed
        %   0 = Stay in same lane
        %   1 = Change to left lane
        %  -1 = Change to right lane
        currentManeuver
        % Lateral destionation after executing maneuver
        y_f
        % Time to start maneuver
        t_start
    end
    
    methods
        function obj = LocalTrajectoryPlanner(varargin)
            %WAYPOINTGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.currentManeuver = 0; % Initially do not execute any maneuver
        end
        
        function initialiseManeuver(obj, y_ego, y_f, maneuver, deltaManeuver, clock)
            % Initialise lane changing maneuver and calculate reference
            % trajectory
            obj.y_f = y_f;
            obj.calculateLaneChangingTrajectoryCoefficients(y_ego, y_f, deltaManeuver);
            obj.currentManeuver = maneuver;
            obj.t_start = clock;
        end
        
        function calculateLaneChangingTrajectoryCoefficients(obj, y_ego, y_f, deltaManeuver)
            % Calculate coefficients for minimum jerk trajectory
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % y_i before lane change
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0


            t_f = deltaManeuver; % deltaT: time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % y_f
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [y_ego; 0; 0; y_f; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
        end
                
        function [updatedPathPoints_Cartesian, yawAngle_in_Cartesian] = Frenet2Cartesian(~,s,laneChangingPoints,currentTrajectory)
            route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
            radian = currentTrajectory(3,1);
            cclockwise = currentTrajectory(4,1);
            yawAngle_in_Cartesian = 0; % NOT IMPLEMENTED FOR CURVED ROAD?
            
            if radian == 0 % Straight road
                route_Vector = route(2,:)-route(1,:);
                route_UnitVector = route_Vector/norm(route_Vector);
                yawAngle_in_Cartesian = atan2d(route_UnitVector(2),route_UnitVector(1));
                sideVector = [cosd(yawAngle_in_Cartesian+90) sind(yawAngle_in_Cartesian+90)];
                
                % Lane Changing Points were already in Frenet - only "s" value should be added
                % "d" is already the reference
                updatedPathPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];
                
                updatedPathPoints_Cartesian = updatedPathPoints_Frenet(:,1)*route_UnitVector+updatedPathPoints_Frenet(:,2)*sideVector+route(1,:);
            else % Curved road
                updatedPathPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];
                all_s = updatedPathPoints_Frenet(:,1);
                all_d = updatedPathPoints_Frenet(:,2);
                
                startPoint = route(1,:);
                rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center
                r = norm(startPoint-rotationCenter); % Get the radius of the rotation
                
                startPointVector = startPoint-rotationCenter;% Vector pointing from the rotation point to the start
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1));
                l = r+(all_d*cclockwise);%current distance from rotation center to position
                lAng = all_s/r+startPointVectorAng;% the angle of vector l
                updatedPathPoints_Cartesian = l.*[cos(lAng) sin(lAng)]+rotationCenter;% the positions in Cartesian
            end
        end
        
        function [s, d] = Cartesian2Frenet(~, currentTrajectory, Vpos_C)
            %Transform a position in Cartesian coordinate into Frenet coordinate
            
            %Function Inputs:
            %route:                 2x2 array [x_s y_s;x_e y_e] the starting point and the ending point of the road
            %vehiclePos_Cartesian:  1x2 array [x y] in Cartesian coordinate
            %radian:                The angle of the whole curved road, positive for counterclockwise turn
            
            %Function Output:
            %yawAngle_in_Cartesian: The angle of the tangent vector on the reference roadline(d=0)
            %s:                     Traversed length along the reference roadline
            %d:                     Lateral offset - positive d means to the left of the reference road 
            
            route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];%Start- and endpoint of the current route
            radian = currentTrajectory(3,1);%radian of the curved road, is 0 for straight road
            Route_StartPoint = route(1,:);
            Route_endPoint = route(2,:);
            cclockwise = currentTrajectory(4,1);
            
            
            if radian == 0%straight road
                route_Vector = Route_endPoint-Route_StartPoint;
                route_UnitVector = route_Vector/norm(route_Vector);
                posVector = Vpos_C-Route_StartPoint; % Vector pointing from the route start point to the vehicle
                
                % Calculate "s" the longitudinal traversed distance
                s = dot(posVector,route_UnitVector);% the projection of posVector on route_UnitVector
                
                % Calculate "d" the lateral distance to the reference road frame
                yawAngle_in_Cartesian = atan2d(route_UnitVector(2),route_UnitVector(1));% orientation angle of the vehicle in Cartesian Coordinate
                sideVector = [cosd(yawAngle_in_Cartesian+90) sind(yawAngle_in_Cartesian+90)];% side vector is perpendicular to the route
                
                d = dot(posVector,sideVector);% the projection of posVector on sideVector - positive d value means to the left
                
            else % Curved Road
                
                rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center
                r = norm(Route_StartPoint-rotationCenter); % Get the radius of the rotation
                startPointVector = Route_StartPoint-rotationCenter;% vector OP_1 in Frenet.xml
                
                
                posVector = Vpos_C-rotationCenter;% the vector from rotation center to position
                d=(norm(posVector)-r)*cclockwise;
                              
                angle = real(acos(dot(posVector,startPointVector)/(norm(posVector)*norm(startPointVector))));

                s = angle*r;
            end
        end
        
    end
end

