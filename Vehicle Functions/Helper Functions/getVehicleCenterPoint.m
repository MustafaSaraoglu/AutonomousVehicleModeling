function centerVehicle = getVehicleCenterPoint(poseRearAxle, wheelBase)
% Get the vehicle's center point by transforming the vehicle's location 
% which is defined as the center of the vehicle's rear axle

    x_RearAxle = poseRearAxle(1, :);
    y_RearAxle = poseRearAxle(2, :);
    yaw = poseRearAxle(3, :);
    
    x_centerVehicle = x_RearAxle + 1/2*wheelBase.*cos(yaw);
    y_centerVehicle = y_RearAxle + 1/2*wheelBase.*sin(yaw);
    
    centerVehicle = [x_centerVehicle; y_centerVehicle];
end
