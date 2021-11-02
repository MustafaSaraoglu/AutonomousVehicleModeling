classdef PlotDrivingScenario< matlab.System
% Plot driving scenario
    
    properties(Nontunable)
        dimensionsVehicle1 % Dimensions (length, width) vehicle 1
        wheelBaseVehicle1 % Wheel base vehicle 1
        dimensionsVehicle2 % Dimensions (length, width) vehicle 2
        wheelBaseVehicle2 % Wheel base vehicle 2
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
    end

    % Pre-computed constants
    properties(Access = private)
        
        % Different plots in the figure
        
        % Vehicles
        plotVehicle1
        plotVehicleLocation1
        plotVehicle2
        plotVehicleLocation2
        
        % Trajectory
        plotTrajectory
        plotWPs
    end
    
    methods(Static)
        function lineOffset = plotRoadLine(x_line, y_line, lineNr, lineOffset, laneWidth)
        % Plot current road line, return offset for next line

            lineRepresentation = '-';
            if lineNr == 2 % Dashed for middle line
                lineRepresentation = '--';
            end

            plot(x_line, y_line, lineRepresentation, 'Color', 'black');

            lineOffset = lineOffset + laneWidth; % Next line
        end
        
        function [plotVehicle, plotVehicleLocation] = plotVehicle(poseRearAxle, wheelBase, dim, color, originRepresentation)
        % Plot vehicle

            centerPoint = getVehicleCenterPoint(poseRearAxle, wheelBase);

            [corners_x, corners_y, ~] = createRectangleVehicle(centerPoint, poseRearAxle(3), dim);

            plotVehicle = plot(corners_x, corners_y, color); % Vehicle rectangle
            plotVehicleLocation = plot(poseRearAxle(1), poseRearAxle(2), originRepresentation, 'Color', color); % Vehicle location
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            close all;
            figure('units','normalized','outerposition',[0 0 1 1]);
            grid on;
            axis equal;
            hold on;
            obj.plotRoad(obj.RoadTrajectory, obj.LaneWidth); % Road 
        end

        function stepImpl(obj, poseVehicle1, egoTrajectory, nextWPsPurePursuit, poseVehicle2)
        % Plot driving scenario
        
             obj.deletePreviousPlots; 
        
            % Vehicle 1
            [obj.plotVehicle1, obj.plotVehicleLocation1] = obj.plotVehicle(poseVehicle1, obj.wheelBaseVehicle1, obj.dimensionsVehicle1, 'cyan', 'o');
            plot(poseVehicle1(1), poseVehicle1(2), '.', 'Color', 'cyan'); % Traces

            % Vehicle 2
            [obj.plotVehicle2, obj.plotVehicleLocation2] = obj.plotVehicle(poseVehicle2, obj.wheelBaseVehicle2, obj.dimensionsVehicle2, 'red', 'o');
            plot(poseVehicle2(1), poseVehicle2(2), '.', 'Color', 'red'); 
            
            % Trajectory
            obj.plotTrajectory = plot(egoTrajectory(:, 1), egoTrajectory(:, 2), 'Color', 'green');
            if size(nextWPsPurePursuit ,2) == 2
                obj.plotWPs = plot(nextWPsPurePursuit(:, 1), nextWPsPurePursuit(:, 2), '-*', 'Color', 'magenta');
            end

            % Adjust Axis
            x2 = poseVehicle2(1);
            y2 = poseVehicle2(2);
            axis([x2-40, x2+40, y2-20, y2+20]); % Camera following V2 as ego vehicle
        end
        
        function deletePreviousPlots(obj)
        % Delete plots (cars, trajectory, etc.) from the previous iteration
            
            % Vehicles
            delete(obj.plotVehicle1);
            delete(obj.plotVehicleLocation1);
            delete(obj.plotVehicle2);
            delete(obj.plotVehicleLocation2);
            
            % Trajectory
            delete(obj.plotTrajectory);
            delete(obj.plotWPs);
        end
        
        function plotRoad(obj, roadTrajectory, laneWidth)
        % Plot road

            route = roadTrajectory([1, 2],[1, 3]).*[1, -1; 1, -1];
            radian = roadTrajectory(3, 1);
            startPoint = route(1, :);
            endPoint = route(2, :);

            lineOffset = -laneWidth/2; % Offset for each line: Start with lower line

            if radian == 0 % Straight road
                route_Vector = endPoint - startPoint;
                routeUnitVector = route_Vector/norm(route_Vector);% unit vector of the route_vector
                routeNormalUnitVector = [-routeUnitVector(2), routeUnitVector(1)]; % Route normal unit vector

                for lineNr = 1:3 % For lower, middle and upper line
                    startLine = startPoint + lineOffset*routeNormalUnitVector;
                    endLine = endPoint + lineOffset*routeNormalUnitVector;

                    lineOffset = obj.plotRoadLine([startLine(1), endLine(1)], [startLine(2), endLine(2)], lineNr, lineOffset, laneWidth);
                end
            else % Curved road
                rotationCenter = roadTrajectory(3, [2, 3]).*[1, -1]; 
                startPointVector = startPoint - rotationCenter;
                routeRadius = norm(startPointVector); 
                startPointVectorAng = atan2(startPointVector(2),startPointVector(1)); 

                angles = startPointVectorAng:radian/(0.2*routeRadius*radian/pi):startPointVectorAng+radian; % Discretise angles from start to end point

                for lineNr = 1:3 % For lower, middle and upper line
                    x_arc = rotationCenter(1) + (routeRadius-lineOffset)*cos(angles);
                    y_arc = rotationCenter(2) + (routeRadius-lineOffset)*sin(angles);

                    lineOffset = obj.plotRoadLine(x_arc, y_arc, lineNr, lineOffset, laneWidth);
                end  
            end
        end
        
        function sts = getSampleTimeImpl(obj)
            % Example: specify discrete sample time
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", 0.1); % For smoother plotting
        end   
    end
end
