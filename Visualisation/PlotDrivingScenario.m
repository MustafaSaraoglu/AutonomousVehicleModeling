classdef PlotDrivingScenario< matlab.System
% Plot driving scenario
    
    properties(Nontunable)
        dimensionsLead % Dimensions (length, width) leading vehicle [[m]; [m]]
        wheelBaseLead % Wheel base leading vehicle [m]
        dimensionsEgo % Dimensions (length, width) ego vehicle [[m]; [m]]
        wheelBaseEgo % Wheel base ego vehicle [m]
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        spaceDiscretisation % Space Discretisation
    end

    % Pre-computed constants
    properties(Access = private)
        
        % Different plots in the figure
        
        % Vehicles
        plotLead
        plotLocationLead
        plotLeadFuture_min
        plotLocationLeadFuture_min
        plotLeadFuture_max
        plotLocationLeadFuture_max
        plotEgo
        plotLocationEgo
        
        % Trajectory
        plotTrajectoryEgo
        plotWPsEgo
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

            plotVehicle = plot(corners_x, corners_y, 'Color', color); % Vehicle rectangle
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
            obj.plotRoad(obj.RoadTrajectory, obj.LaneWidth);
%             obj.plotDiscreteSpace(); 
        end

        function stepImpl(obj, poseLead, poseLeadFuture_min, poseLeadFuture_max, egoTrajectory, nextWPsPurePursuit, poseEgo)
        % Plot driving scenario
        
             obj.deletePreviousPlots; 
        
            % Vehicle Lead
            [obj.plotLead, obj.plotLocationLead] = obj.plotVehicle(poseLead, obj.wheelBaseLead, obj.dimensionsLead, [0, 204/255, 204/255], 'o');
            plot(poseLead(1), poseLead(2), '.', 'Color', 'cyan'); % Traces
            % Future predictions
            [obj.plotLeadFuture_min, obj.plotLocationLeadFuture_min] = obj.plotVehicle(poseLeadFuture_min, obj.wheelBaseLead, obj.dimensionsLead, [153/255, 1, 1], 'o');
            [obj.plotLeadFuture_max, obj.plotLocationLeadFuture_max] = obj.plotVehicle(poseLeadFuture_max, obj.wheelBaseLead, obj.dimensionsLead, [153/255, 1, 1], 'o');

            % Vehicle Ego
            [obj.plotEgo, obj.plotLocationEgo] = obj.plotVehicle(poseEgo, obj.wheelBaseEgo, obj.dimensionsEgo, 'red', 'o');
            plot(poseEgo(1), poseEgo(2), '.', 'Color', 'red'); 
            
            % Trajectory
            obj.plotTrajectoryEgo = plot(egoTrajectory(:, 1), egoTrajectory(:, 2), 'Color', 'green');
            if size(nextWPsPurePursuit ,2) == 2
                obj.plotWPsEgo = plot(nextWPsPurePursuit(:, 1), nextWPsPurePursuit(:, 2), '-*', 'Color', 'magenta');
            end

            % Adjust Axis
            x2 = poseEgo(1);
            y2 = poseEgo(2);
            axis([x2-40, x2+40, y2-20, y2+20]); % Camera following V2 as ego vehicle
        end
        
        function deletePreviousPlots(obj)
        % Delete plots (cars, trajectory, etc.) from the previous iteration
            
            % Vehicles
            delete(obj.plotLead);
            delete(obj.plotLocationLead);
            delete(obj.plotLeadFuture_min);
            delete(obj.plotLocationLeadFuture_min);
            delete(obj.plotLeadFuture_max);
            delete(obj.plotLocationLeadFuture_max);
            delete(obj.plotEgo);
            delete(obj.plotLocationEgo);
            
            % Trajectory
            delete(obj.plotTrajectoryEgo);
            delete(obj.plotWPsEgo);
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
        
        function plotDiscreteSpace(obj)
        % Plot discrete space
            
            for row = 1:size(obj.spaceDiscretisation, 1)
                for column = 1:size(obj.spaceDiscretisation, 2)
                    cell_Frenet = obj.spaceDiscretisation{row, column};
                    corners_Frenet = [cell_Frenet(1,1), cell_Frenet(1,1), cell_Frenet(1,2), cell_Frenet(1,2); cell_Frenet(2,1), cell_Frenet(2,2), cell_Frenet(2,1), cell_Frenet(2,2)];
                    corners_Cartesian = (Frenet2Cartesian(0, corners_Frenet', obj.RoadTrajectory))';
                    plot(corners_Cartesian(1, :), corners_Cartesian(2, :), 'x', 'Color', 'blue');
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
