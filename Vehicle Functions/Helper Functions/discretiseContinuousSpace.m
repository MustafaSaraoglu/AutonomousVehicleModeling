function spaceDiscretisation = discretiseContinuousSpace(roadTrajectory, laneWidth, cell_length, laneCell_width)
% Divide road into discrete cells using Frenet Coordinates 
% return cell array containing the corner points of each discrete cell
% represented by the corresponding row and column index in the cell array

    route = roadTrajectory([1, 2],[1, 3]).*[1, -1; 1, -1];
    radian = roadTrajectory(3, 1);
    startPoint = route(1, :);

    if radian == 0 % Straight road
        endPoint = route(2, :);
        route_Vector = endPoint - startPoint;

        routeLength = norm(route_Vector);
    else % Curved road
        rotationCenter = roadTrajectory(3, [2, 3]).*[1, -1]; 
        startPointVector = startPoint - rotationCenter;
        routeRadius = norm(startPointVector); 

        routeLength = abs(radian*routeRadius);
    end

    % Cell dimensions
    roadBoundryCell_width = (laneWidth - laneCell_width)/2; % Width of road boundry cells [m]
    roadCenterCell_width = 2*roadBoundryCell_width; % Width of road center cell [m]

    number_rows = ceil(routeLength/cell_length);
    number_columns = 5; % Divide road width into 5 cells
    spaceDiscretisation = cell(number_rows, number_columns); % Preallocate spaceDiscretisation cell array

    for row = 1:number_rows
        s_start = (row-1)*cell_length;
        s_end = s_start + cell_length;
        if row == number_rows % Check last row in case could not divide perfectly into equal sized cells
            s_end = routeLength;
        end

        for column = 1:number_columns
            switch column
                case 1 % Right road boundry cell
                    d_start = -laneWidth/2; % Start at -laneWidth/2
                    d_end = -laneWidth/2 + roadBoundryCell_width;
                case 2 % Right lane cell
                    d_start = -laneWidth/2 + roadBoundryCell_width;
                    d_end = -laneWidth/2 + roadBoundryCell_width + laneCell_width;
                case 3 % Center of the road cell
                    d_start = -laneWidth/2 + roadBoundryCell_width + laneCell_width;
                    d_end = -laneWidth/2 + roadBoundryCell_width + laneCell_width + roadCenterCell_width;
                case 4 % Left lane cell
                    d_start = -laneWidth/2 + roadBoundryCell_width + laneCell_width + roadCenterCell_width;
                    d_end = -laneWidth/2 + roadBoundryCell_width + 2*laneCell_width + roadCenterCell_width;
                case 5 % Left road boundry cell
                    d_start = -laneWidth/2 + roadBoundryCell_width + 2*laneCell_width + roadCenterCell_width;
                    d_end = -laneWidth/2 + 2*roadBoundryCell_width + 2*laneCell_width + roadCenterCell_width;
            end

            spaceDiscretisation{row, column} = [s_start, s_end; d_start, d_end];
        end
    end
end
