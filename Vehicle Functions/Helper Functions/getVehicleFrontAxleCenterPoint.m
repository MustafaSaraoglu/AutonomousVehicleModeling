function centerFrontAxle = getVehicleFrontAxleCenterPoint(poseRearAxle, wheelBase)
% Get the center of the vehicle's front axle by transforming the vehicle's location

    x_RearAxle = poseRearAxle(1);
    y_RearAxle = poseRearAxle(2);
    yaw = poseRearAxle(3);
    
    x_centerFrontAxle = x_RearAxle + wheelBase*cos(yaw);
    y_centerFrontAxle = y_RearAxle + wheelBase*sin(yaw);
    
    centerFrontAxle = [x_centerFrontAxle, y_centerFrontAxle];
end

