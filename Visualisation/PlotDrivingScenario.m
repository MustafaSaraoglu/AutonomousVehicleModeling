classdef PlotDrivingScenario< matlab.System
% Plot driving scenario
    
    properties(Nontunable)
        dimensionsLead % Dimensions (length, width) leading vehicle [[m]; [m]]
        wheelBaseLead % Wheel base leading vehicle [m]
        dimensionsEgo % Dimensions (length, width) ego vehicle [[m]; [m]]
        wheelBaseEgo % Wheel base ego vehicle [m]
        
        s_max % Maximum allowed longitudinal postion [m]
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        spaceDiscretisationMatrix % Space Discretisation Matrix
    end

    % Pre-computed constants
    properties(Access = private)
        plots2update % Plots to update at every time step
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
        end

        function stepImpl(obj, poseLead, poseLeadFuture_min, poseLeadFuture_max, positionEgoFuture, egoReachability, nextWPsPurePursuit, poseEgo)
        % Plot driving scenario
        
            obj.deletePreviousPlots; 
        
            % Vehicle Lead
            [obj.plots2update.vehicles.Lead, obj.plots2update.vehicles.LocationLead] = obj.plotVehicle(poseLead, obj.wheelBaseLead, obj.dimensionsLead, [0, 204/255, 204/255], 'o');
            plot(poseLead(1), poseLead(2), '.', 'Color', 'cyan'); % Traces
            % Future predictions
            [obj.plots2update.vehicles.LeadFuture_min, obj.plots2update.vehicles.LocationLeadFuture_min] = obj.plotVehicle(poseLeadFuture_min, obj.wheelBaseLead, obj.dimensionsLead, [153/255, 1, 1], 'o');
            [obj.plots2update.vehicles.LeadFuture_max, obj.plots2update.vehicles.LocationLeadFuture_max] = obj.plotVehicle(poseLeadFuture_max, obj.wheelBaseLead, obj.dimensionsLead, [153/255, 1, 1], 'o');

            % Vehicle Ego
            [obj.plots2update.vehicles.Ego, obj.plots2update.vehicles.LocationEgo] = obj.plotVehicle(poseEgo, obj.wheelBaseEgo, obj.dimensionsEgo, 'red', 'o');
            plot(poseEgo(1), poseEgo(2), '.', 'Color', 'red'); 
            obj.plots2update.vehicles.EgoFuture = plot(positionEgoFuture(1), positionEgoFuture(2), '*', 'Color', 'green');
            
            % Trajectory
            if size(nextWPsPurePursuit ,2) == 2
                obj.plots2update.trajectory.WPsEgo = plot(nextWPsPurePursuit(:, 1), nextWPsPurePursuit(:, 2), '-*', 'Color', 'magenta');
            end
            
            % Ego Reachability
            obj.plots2update.reachability.EgoReachabilityMinBoundary = plot(egoReachability(:, 1), egoReachability(:, 2), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityMaxBoundary = plot(egoReachability(:, 3), egoReachability(:, 4), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityRightBoundary = plot(egoReachability(:, 5), egoReachability(:, 6), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityLeftBoundary = plot(egoReachability(:, 7), egoReachability(:, 8), 'Color', [51/255, 102/255, 0]);
            obj.plots2update.reachability.EgoReachabilityEmergencyBoundary = plot(egoReachability(:, 9), egoReachability(:, 10), 'Color', [153/255, 0, 0]);

            % Adjust Axis
            x2 = poseEgo(1);
            y2 = poseEgo(2);
            axis([x2-80, x2+80, y2-40, y2+40]); % Camera following V2 as ego vehicle
        end
        
        function deletePreviousPlots(obj)
        % Delete plots (cars, trajectory, etc.) from the previous iteration
            
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
            
            s = 0:0.1:obj.s_max;
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
