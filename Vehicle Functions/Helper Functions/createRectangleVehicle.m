function [corners_x, corners_y, Hitbox] = createRectangleVehicle(centerPoint, yaw, dim)
% Vehicle representation as rectangle using center point

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

    p1r = centerPoint + Rmatrix*p1;
    p2r = centerPoint + Rmatrix*p2;
    p3r = centerPoint + Rmatrix*p3;
    p4r = centerPoint + Rmatrix*p4;

    % Connect points to rectangle
    rectangle = [p1r p2r p3r p4r p1r];
    Hitbox = [p1r p2r p3r p4r];

    corners_x = transpose(rectangle(1, :));
    corners_y = transpose(rectangle(2, :));
end