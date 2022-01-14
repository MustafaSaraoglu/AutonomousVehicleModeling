classdef PlotDrivingScenario< matlab.System
% Plot driving scenario
    
    properties(Nontunable)
        dimensionsOtherVehicles % Dimensions (length, width) other vehicles [[m]; [m]]
        wheelBaseOtherVehicles % Wheel base other vehicles [m]
        dimensionsEgo % Dimensions (length, width) ego vehicle [[m]; [m]]
        wheelBaseEgo % Wheel base ego vehicle [m]
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        spaceDiscretisation % Space Discretisation
    end

    % Pre-computed constants
    properties(Access = private)
        plots2update % Plots and annotations to update at every time step
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
            
            % Plot corner points of discrete cells
            obj.plotDiscreteSpace(); 
            
            obj.drivingModes = ...
                containers.Map([1, 2, 3], {'FreeDrive', 'VehicleFollowing', 'EmergencyBrake'});
        end

        function stepImpl(obj, poseOtherVehicles, poseOtherVehiclesFuture, egoReachability, nextWPsPurePursuit, currentDrivingMode, poseEgo)
        % Plot driving scenario
            
            %% Delete plots from the previous iteration
            obj.deletePreviousPlots; 
            
            %% Other Vehicles
            for id_otherVehicle = 1:size(poseOtherVehicles, 2)
                %% Current position
                % To store plots in struct
                vehicleName = ['OtherVehicle', num2str(id_otherVehicle)];
                
                % Rectangle representation
                [obj.plots2update.(vehicleName).rectangle, obj.plots2update.(vehicleName).location] = obj.plotVehicle(poseOtherVehicles(:, id_otherVehicle), obj.wheelBaseOtherVehicles(id_otherVehicle), obj.dimensionsOtherVehicles(:, id_otherVehicle), [0, 204/255, 204/255], 'o');
                % Traces from previous positions
                plot(poseOtherVehicles(1, id_otherVehicle), poseOtherVehicles(2, id_otherVehicle), '.', 'Color', 'cyan'); 
                
                %% Future predictions 
                [obj.plots2update.(vehicleName).rectangleFuture_min, obj.plots2update.(vehicleName).locationFuture_min] = obj.plotVehicle(poseOtherVehiclesFuture(1:3, id_otherVehicle), obj.wheelBaseOtherVehicles(id_otherVehicle), obj.dimensionsOtherVehicles(:, id_otherVehicle), [153/255, 1, 1], 'o');
                [obj.plots2update.(vehicleName).rectangleFuture_max, obj.plots2update.(vehicleName).locationFuture_max] = obj.plotVehicle(poseOtherVehiclesFuture(4:6, id_otherVehicle), obj.wheelBaseOtherVehicles(id_otherVehicle), obj.dimensionsOtherVehicles(:, id_otherVehicle), [153/255, 1, 1], 'o');
            end
            
            %% Ego Vehicle
            [obj.plots2update.EgoVehicle.rectangle, obj.plots2update.EgoVehicle.location] = obj.plotVehicle(poseEgo, obj.wheelBaseEgo, obj.dimensionsEgo, 'red', 'o');
            % Traces from previous positions
            plot(poseEgo(1), poseEgo(2), '.', 'Color', 'red'); 
            
            % Trajectory
            % Only plot if lateral controller is Pure Pursuit
            if size(nextWPsPurePursuit, 2) == 2
                obj.plots2update.trajectory.WPsEgo = plot(nextWPsPurePursuit(:, 1), nextWPsPurePursuit(:, 2), '-*', 'Color', 'magenta');
            end
            
            %% Driving mode and current state
            obj.plots2update.annotations.drivingMode = annotation('textbox',[.5 .72 .2 .2], 'String', ['Current Driving Mode: ' obj.drivingModes(currentDrivingMode)], 'EdgeColor','none');
            
            %% Ego Reachability
            % Min and max boundary curve
            obj.plots2update.reachability.EgoReachabilityMinBoundary = plot(egoReachability(:, 1), egoReachability(:, 2), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityMaxBoundary = plot(egoReachability(:, 3), egoReachability(:, 4), 'Color', [51/255, 102/255, 0]);
            
            % Right and left boundary curve
            obj.plots2update.reachability.EgoReachabilityRightBoundary = plot(egoReachability(:, 5), egoReachability(:, 6), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityLeftBoundary = plot(egoReachability(:, 7), egoReachability(:, 8), 'Color', [51/255, 102/255, 0]);
            
            % Connect to area
            obj.connectEgoReachabilityCurves(egoReachability)
            
            % Emergency boundary curve
            obj.plots2update.reachability.EgoReachabilityEmergencyBoundary = plot(egoReachability(:, 9), egoReachability(:, 10), 'Color', [153/255, 0, 0]);

            %% Adjust Axis
            xEgo = poseEgo(1);
            yEgo = poseEgo(2);
            axis([xEgo-80, xEgo+80, yEgo-40, yEgo+40]); % Camera following ego vehicle
        end
        
        function deletePreviousPlots(obj)
        % Delete plots (cars, trajectory, etc.) and annotations from the previous iteration
            
            if isempty(obj.plots2update) % No plots for first iteration
                return
            end
            
            plotCategories= fieldnames(obj.plots2update);
            
            % Delete all plot objects in plot struct plots2update
            for category = 1:numel(plotCategories)
                plotStruct = obj.plots2update.(plotCategories{category});
                plots = fieldnames(plotStruct);
                for plot = 1:numel(plots)
                    delete(plotStruct.(plots{plot}))
                end
            end
        end
        
        function connectEgoReachabilityCurves(obj, egoReachability)
        % Connect boundary lines from steering reachability to area
             
            % Right boundary curve
            rightBoundary_x = egoReachability(:, 5);
            rightBoundary_y = egoReachability(:, 6);
            
            % Left boundary curve
            leftBoundary_x = egoReachability(:, 7);
            leftBoundary_y = egoReachability(:, 8);
            
            % Reference point to connect is the last point in the left/right boundary curve
            x_lowerReferenceRight = rightBoundary_x(end);
            y_lowerReferenceRight = rightBoundary_y(end);
            x_lowerReferenceLeft = leftBoundary_x(end);
            y_lowerReferenceLeft = leftBoundary_y(end);
            
            % If all points of the right/left boundary curve are the same means,
            % that the right/left boundary curve was deleted due to the turning angle limit,
            % which should prevent the reachable space going behind
            % the ego vehicle's current position (which would allow going backwards)
            if rightBoundary_x(1) == rightBoundary_x(end)
               % Max boundary curve 
                minBoundary_x = egoReachability(:, 1);
                minBoundary_y = egoReachability(:, 2);
                
                % Reference point to connect is the first/last point in the min boundary curve
                x_lowerReferenceRight = minBoundary_x(1);
                y_lowerReferenceRight = minBoundary_y(2);
                x_lowerReferenceLeft = minBoundary_x(end);
                y_lowerReferenceLeft = minBoundary_y(end);
            end
            
            % Max boundary curve 
            maxBoundary_x = egoReachability(:, 3);
            maxBoundary_y = egoReachability(:, 4);
            
            % Connect reference point to most right (first) point of max boundary curve to
            obj.plots2update.reachability.EgoReachabilityConnectRight = plot([x_lowerReferenceRight, maxBoundary_x(1)], [y_lowerReferenceRight, maxBoundary_y(1)], 'Color', [51/255, 102/255, 0]);
            
            % Connect reference point to most left (last) point of max boundary curve
            obj.plots2update.reachability.EgoReachabilityConnectLeft = plot([x_lowerReferenceLeft, maxBoundary_x(end)], [y_lowerReferenceLeft, maxBoundary_y(end)], 'Color', [51/255, 102/255, 0]);
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
            
            % Create points to draw line
            s = 0:0.1:routeLength;
            d = d*ones(1, length(s));
            
            [lanePoints, ~] = Frenet2Cartesian(s', d', obj.RoadTrajectory);
            
            plot(lanePoints(:, 1), lanePoints(:, 2), lineRepresentation, 'Color', 'black')
        end
        
        function plotDiscreteSpace(obj)
        % Plot discrete space
            
            % Iterate over each cell
            for row = 1:size(obj.spaceDiscretisation, 1)
                for column = 1:size(obj.spaceDiscretisation, 2)
                    cell_Frenet = obj.spaceDiscretisation{row, column};
                    
                    % Combine limits to find corner points
                    corners_Frenet = (combvec(cell_Frenet(1, :), cell_Frenet(2, :)))';
                    corners_Cartesian = (Frenet2Cartesian(corners_Frenet(:, 1), corners_Frenet(:, 2), obj.RoadTrajectory))';
                    
                    plot(corners_Cartesian(1, :), corners_Cartesian(2, :), 'x', 'Color', 'blue');
                end
            end
        end
        
        function sts = getSampleTimeImpl(obj)
            % Example: specify discrete sample time
            sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', 0.1); % For faster plotting
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
