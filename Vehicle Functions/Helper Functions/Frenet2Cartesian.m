function [refPosition, refOrientation] = Frenet2Cartesian(s, d, currentTrajectory)
    % this function transfer a position in Frenet coordinate into Cartesian coordinate
    % input:
    % route is a 2x2 array [x_s y_s;x_e y_e]contains the startpoint and the endpoint of the road
    % s is the journey on the reference roadline(d=0)
    % d is the vertical offset distance to the reference roadline,positive d means away from center
    % output:
    % position_Cart is the 1x2 array [x y] in Cartesian coordinate
    % orientation_Cart is the angle of the tangent vector on the reference roadline and the x axis 
    % of cartesian detail information check Frenet.mlx

    route = currentTrajectory([1, 2], [1, 3]).*[1, -1; 1, -1];
    radian = currentTrajectory(3, 1); % radian of the whole curved road, is positive when 
                                      % counterclockwise turns
    cclockwise = currentTrajectory(4, 1);

    if radian == 0
        route_Vector = route(2, :) - route(1, :);
        route_UnitVector = route_Vector/norm(route_Vector);
        % Fast rotation by 90 degrees to find the normal vector  
        normalVector = [-route_UnitVector(2), route_UnitVector(1)];

        refPosition = s*route_UnitVector + d*normalVector + route(1, :);
        % reverse tangent of unit vector
        refOrientation = atan2(route_UnitVector(2), route_UnitVector(1))*ones(length(s), 1); 
    else
        startPoint = route(1,:);
        rotationCenter = currentTrajectory(3, [2, 3]).*[1, -1]; % Get the rotation center

        startPointVector = startPoint - rotationCenter; % Vector pointing from the rotation point 
                                                        % to the start
        r = norm(startPointVector); % Get the radius of the rotation

        startPointVectorAng = atan2(startPointVector(2), startPointVector(1));

        l = r + d*cclockwise;%current distance from rotation center to position
        lAng = startPointVectorAng - cclockwise*s/r;% the angle of vector l
        refPosition = l.*[cos(lAng), sin(lAng)] + rotationCenter;% the positions in Cartesian
        refOrientation = lAng - cclockwise*pi/2;
    end
end