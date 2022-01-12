classdef PlotDrivingScenario< matlab.System
% Plot driving scenario
    
    properties(Nontunable)
        dimensionsOtherVehicles % Dimensions (length, width) other vehicles [[m]; [m]]
        wheelBaseOtherVehicles % Wheel base other vehicles [m]
        dimensionsEgo % Dimensions (length, width) ego vehicle [[m]; [m]]
        wheelBaseEgo % Wheel base ego vehicle [m]
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        spaceDiscretisationMatrix % Space Discretisation Matrix
    end

    % Pre-computed constants
    properties(Access = private)
        plots2update % Plots to update at every time step
        drivingModes % Possible driving modes
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            close all;
            figure('units','normalized','outerposition',[0 0 1 1]);
            grid on;
            axis equal;
            hold on;
            obj.plotRoadLine(-obj.LaneWidth/2, '-'); % Right
            obj.plotRoadLine(obj.LaneWidth/2, '--'); % Center
            obj.plotRoadLine(3*obj.LaneWidth/2, '-'); % Left
            obj.plotDiscreteSpace(); 
            
            obj.drivingModes = ...
                containers.Map([1, 2, 3], {'FreeDrive', 'VehicleFollowing', 'EmergencyBrake'});
        end

        function stepImpl(obj, poseOtherVehicles, poseOtherVehiclesFuture, positionEgoFuture, egoReachability, nextWPsPurePursuit, currentDrivingMode, poseEgo)
        % Plot driving scenario
        
            obj.deletePreviousPlots; 
            
            % Other Vehicles
            for id_otherVehicle = 1:size(poseOtherVehicles, 2)
                % Vehicles
                fieldVehicleName = append('OtherVehicle', num2str(id_otherVehicle));
                fieldVehicleLocation = append('LocationOtherVehicle', num2str(id_otherVehicle));
                [obj.plots2update.vehicles.(fieldVehicleName), obj.plots2update.vehicles.(fieldVehicleLocation)] = obj.plotVehicle(poseOtherVehicles(:, id_otherVehicle), obj.wheelBaseOtherVehicles(id_otherVehicle), obj.dimensionsOtherVehicles(:, id_otherVehicle), [0, 204/255, 204/255], 'o');
                plot(poseOtherVehicles(1, id_otherVehicle), poseOtherVehicles(2, id_otherVehicle), '.', 'Color', 'cyan'); % Traces
                % Future predictions
                fieldVehicleFutureMinName = append('OtherVehicleFuture_min', num2str(id_otherVehicle));
                fieldVehicleFutureMinLocation = append('LocationOtherVehicleFuture_min', num2str(id_otherVehicle));
                fieldVehicleFutureMaxName = append('OtherVehicleFuture_max', num2str(id_otherVehicle));
                fieldVehicleFutureMaxLocation = append('LocationOtherVehicleFuture_max', num2str(id_otherVehicle));
                [obj.plots2update.vehicles.(fieldVehicleFutureMinName), obj.plots2update.vehicles.(fieldVehicleFutureMinLocation)] = obj.plotVehicle(poseOtherVehiclesFuture(1:3, id_otherVehicle), obj.wheelBaseOtherVehicles(id_otherVehicle), obj.dimensionsOtherVehicles(:, id_otherVehicle), [153/255, 1, 1], 'o');
                [obj.plots2update.vehicles.(fieldVehicleFutureMaxName), obj.plots2update.vehicles.(fieldVehicleFutureMaxLocation)] = obj.plotVehicle(poseOtherVehiclesFuture(4:6, id_otherVehicle), obj.wheelBaseOtherVehicles(id_otherVehicle), obj.dimensionsOtherVehicles(:, id_otherVehicle), [153/255, 1, 1], 'o');
            end
            
            % Vehicle Ego
            [obj.plots2update.vehicles.Ego, obj.plots2update.vehicles.LocationEgo] = obj.plotVehicle(poseEgo, obj.wheelBaseEgo, obj.dimensionsEgo, 'red', 'o');
            plot(poseEgo(1), poseEgo(2), '.', 'Color', 'red'); 
            obj.plots2update.vehicles.EgoFuture = plot(positionEgoFuture(1), positionEgoFuture(2), '*', 'Color', 'green');
            
            % Trajectory
            if size(nextWPsPurePursuit ,2) == 2
                obj.plots2update.trajectory.WPsEgo = plot(nextWPsPurePursuit(:, 1), nextWPsPurePursuit(:, 2), '-*', 'Color', 'magenta');
            end
            
            % Driving mode and current state
            obj.plots2update.annotations.drivingMode = annotation('textbox',[.5 .72 .2 .2], 'String', ['Current Driving Mode: ' obj.drivingModes(currentDrivingMode)], 'EdgeColor','none');
            
            % Ego Reachability
            obj.plots2update.reachability.EgoReachabilityMinBoundary = plot(egoReachability(:, 1), egoReachability(:, 2), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityMaxBoundary = plot(egoReachability(:, 3), egoReachability(:, 4), 'Color', [51/255, 102/255, 0]);
            % Temp: Connect limiting curves to area
            x_lowerReferenceRight = egoReachability(end, 5);
            y_lowerReferenceRight = egoReachability(end, 6);
            x_lowerReferenceLeft = egoReachability(end, 7);
            y_lowerReferenceLeft = egoReachability(end, 8);
            if egoReachability(1, 5) == egoReachability(end, 5)
                x_lowerReferenceRight = egoReachability(1, 1);
                y_lowerReferenceRight = egoReachability(1, 2);
                x_lowerReferenceLeft = egoReachability(end, 1);
                y_lowerReferenceLeft = egoReachability(end, 2);
            end
            obj.plots2update.reachability.EgoReachabilityConnectRight = plot([x_lowerReferenceRight, egoReachability(1, 3)], [y_lowerReferenceRight, egoReachability(1, 4)], 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityConnectLeft = plot([x_lowerReferenceLeft, egoReachability(end, 3)], [y_lowerReferenceLeft, egoReachability(end, 4)], 'Color', [51/255, 102/255, 0]);
            
            obj.plots2update.reachability.EgoReachabilityRightBoundary = plot(egoReachability(:, 5), egoReachability(:, 6), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityLeftBoundary = plot(egoReachability(:, 7), egoReachability(:, 8), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityEmergencyBoundary = plot(egoReachability(:, 9), egoReachability(:, 10), 'Color', [153/255, 0, 0]);

            % Adjust Axis
            xEgo = poseEgo(1);
            yEgo = poseEgo(2);
            axis([xEgo-80, xEgo+80, yEgo-40, yEgo+40]); % Camera following ego vehicle
        end
        
        function deletePreviousPlots(obj)
        % Delete plots (cars, trajectory, etc.) and annotations from the previous iteration
            
            if isempty(obj.plots2update)
                return
            end
            plotCategories= fieldnames(obj.plots2update);
            for category = 1:numel(plotCategories)
                plotStruct = obj.plots2update.(plotCategories{category});
                plots = fieldnames(plotStruct);
                for plot = 1:numel(plots)
                    delete(plotStruct.(plots{plot}))
                end
            end
        end
        
        function plotRoadLine(obj, d, lineRepresentation)
        % Plot road line according to lateral position
            
            route = obj.RoadTrajectory([1, 2],[1, 3]).*[1, -1; 1, -1];
            radian = obj.RoadTrajectory(3, 1);
            startPoint = route(1, :);

            if radian == 0 % Straight road
                endPoint = route(2, :);
                route_Vector = endPoint - startPoint;

                routeLength = norm(route_Vector);
            else % Curved road
                rotationCenter = obj.RoadTrajectory(3, [2, 3]).*[1, -1]; 
                startPointVector = startPoint - rotationCenter;
                routeRadius = norm(startPointVector); 

                routeLength = abs(radian*routeRadius);
            end
            s = 0:0.1:routeLength;
            d = d*ones(1, length(s));
            [lanePoints, ~] = Frenet2Cartesian(s', d', obj.RoadTrajectory);
            plot(lanePoints(:, 1), lanePoints(:, 2), lineRepresentation, 'Color', 'black')
        end
        
        function plotDiscreteSpace(obj)
        % Plot discrete space
            
            for row = 1:size(obj.spaceDiscretisationMatrix, 1)/2
                for column = 1:size(obj.spaceDiscretisationMatrix, 2)/2
                    cell_Frenet = obj.spaceDiscretisationMatrix(2*row-1:2*row, 2*column-1:2*column);
                    corners_Frenet = (combvec(cell_Frenet(1, :), cell_Frenet(2, :)))';
                    corners_Cartesian = (Frenet2Cartesian(corners_Frenet(:, 1), corners_Frenet(:, 2), obj.RoadTrajectory))';
                    plot(corners_Cartesian(1, :), corners_Cartesian(2, :), 'x', 'Color', 'blue');
                end
            end
        end
        
        function sts = getSampleTimeImpl(obj)
            % Example: specify discrete sample time
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", 0.1); % For faster plotting
        end   
    end
    
    methods(Static)        
        function [plotVehicle, plotVehicleLocation] = plotVehicle(poseRearAxle, wheelBase, dim, color, originRepresentation)
        % Plot vehicle

            centerPoint = getVehicleCenterPoint(poseRearAxle, wheelBase);

            [corners_x, corners_y, ~] = createRectangleVehicle(centerPoint, poseRearAxle(3), dim);

            plotVehicle = plot(corners_x, corners_y, 'Color', color); % Vehicle rectangle
            plotVehicleLocation = plot(poseRearAxle(1), poseRearAxle(2), originRepresentation, 'Color', color); % Vehicle location
        end
    end
end
