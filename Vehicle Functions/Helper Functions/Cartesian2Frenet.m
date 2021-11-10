function [s, d] = Cartesian2Frenet(currentTrajectory, Vpos_C)
%% FROM MOBATSIM

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