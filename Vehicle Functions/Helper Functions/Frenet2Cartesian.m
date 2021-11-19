function [updatedPathPoints_Cartesian, refOrientation] = Frenet2Cartesian(s, d, currentTrajectory)
%% FROM MOBATSim (Modified: Added reference orientation and other modifications)

    route = currentTrajectory([1, 2], [1, 3]).*[1, -1; 1, -1];
    radian = currentTrajectory(3, 1);
    cclockwise = currentTrajectory(4, 1);

    if radian == 0
        route_Vector = route(2, :) - route(1, :);
        route_UnitVector = route_Vector/norm(route_Vector);
        normalVector = [-route_UnitVector(2), route_UnitVector(1)];% Fast rotation by 90 degrees to find the normal vector  

        updatedPathPoints_Cartesian = s*route_UnitVector + d*normalVector + route(1, :);
        refOrientation = atan2(route_UnitVector(2), route_UnitVector(1))*ones(length(s), 1); % reverse tangent of unit vector
    else
        startPoint = route(1,:);
        rotationCenter = currentTrajectory(3, [2, 3]).*[1, -1]; % Get the rotation center

        startPointVector = startPoint - rotationCenter;% Vector pointing from the rotation point to the start
        r = norm(startPointVector); % Get the radius of the rotation

        startPointVectorAng = atan2(startPointVector(2), startPointVector(1));

        l = r + (d*cclockwise);%current distance from rotation center to position
        lAng = -cclockwise*s/r + startPointVectorAng;% the angle of vector l
        updatedPathPoints_Cartesian = l.*[cos(lAng), sin(lAng)] + rotationCenter;% the positions in Cartesian
        refOrientation = lAng - cclockwise*pi/2;
    end
end