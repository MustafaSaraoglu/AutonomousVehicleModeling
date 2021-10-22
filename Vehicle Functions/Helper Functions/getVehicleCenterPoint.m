function centerPoint = getVehicleCenterPoint(poseRearAxle, wheelBase)
    % Get the vehicle's center point (x_center, y_center) by transforming the vehicles location
    % which is defined as the center of the vehicle's rear axle
    
    x_RearAxle = poseRearAxle(1);
    y_RearAxle = poseRearAxle(2);
    yaw = poseRearAxle(3);
    
    x_center = x_RearAxle + 1/2*wheelBase*cos(yaw);
    y_center = y_RearAxle + 1/2*wheelBase*sin(yaw);
    
    centerPoint = [x_center; y_center];
end