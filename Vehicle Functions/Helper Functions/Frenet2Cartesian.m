function [updatedPathPoints_Cartesian, refOrientation] = Frenet2Cartesian(s,laneChangingPoints,currentTrajectory)
%% FROM MOBATSim (Modified: Added reference orientation)

    route = currentTrajectory([1,2],[1,3]).*[1 -1;1 -1];
    radian = currentTrajectory(3,1);
    cclockwise = currentTrajectory(4,1);

    if radian == 0
        route_Vector = route(2,:)-route(1,:);
        route_UnitVector = route_Vector/norm(route_Vector);
        normalVector = [-route_UnitVector(2),route_UnitVector(1)];% Fast rotation by 90 degrees to find the normal vector  

        % Lane Changing Points were already in Frenet - only "s" value should be added
        % "d" is already the reference
        updatedPathPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];

        updatedPathPoints_Cartesian = updatedPathPoints_Frenet(:,1)*route_UnitVector+updatedPathPoints_Frenet(:,2)*normalVector+route(1,:);
        refOrientation = atan2d(route_UnitVector(2),route_UnitVector(1)); % reverse tangent of unit vector
    else
        updatedPathPoints_Frenet=[s+laneChangingPoints(:,1) laneChangingPoints(:,2)];
        all_s = updatedPathPoints_Frenet(:,1);
        all_d = updatedPathPoints_Frenet(:,2);

        startPoint = route(1,:);
        rotationCenter = currentTrajectory(3,[2 3]).*[1 -1]; % Get the rotation center

        startPointVector = startPoint-rotationCenter;% Vector pointing from the rotation point to the start
        r = norm(startPointVector); % Get the radius of the rotation

        startPointVectorAng = atan2(startPointVector(2),startPointVector(1));

        l = r+(all_d*cclockwise);%current distance from rotation center to position
        lAng = all_s/r+startPointVectorAng;% the angle of vector l
        updatedPathPoints_Cartesian = l.*[cos(lAng) sin(lAng)]+rotationCenter;% the positions in Cartesian
        refOrientation = rad2deg(lAng+(-cclockwise)*pi/2);
    end
%%    
    refOrientation = deg2rad(refOrientation);
end