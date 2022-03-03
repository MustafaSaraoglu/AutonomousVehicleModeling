classdef State
% Continuous Vehicle states 
    
    properties
        s
        d
        orientation
        speed
    end
    
    methods
        function obj = State(s, d, orientation, speed)
            %STATE Construct an instance of this class
            if nargin > 0
                obj.s = s;
                obj.d = d;
                obj.orientation = orientation;
                obj.speed = speed;
            end
        end
        
        function empty = isemptyState(obj)
        % Check whether State is empty
            empty = false;
            if isempty(obj.s) && isempty(obj.s) && isempty(obj.orientation) && isempty(obj.speed)
                empty = true;
            end
        end
    end
    
    methods(Static)
        function stateCombinations = getStateCombinations(states)
        % Get all possible state combinations for each decision of each
        % vehicle
            
            % n_stateCombinations = \prod_{i=1}^{n_vehicles} n_states_{i}
            n_vehicles = size(states, 2);
            
            for id_vehicle = 1:n_vehicles
                % Get number of nonempty states for each vehicle
                n_states = sum(arrayfun(@(x) ~isemptyState(x), states(:, id_vehicle)));
                if id_vehicle == 1
                    id_combinations = 1:n_states;
                else
                    id_combinations = combvec(id_combinations, 1:n_states);
                end
            end
            
            id_combinations = id_combinations';
            
            % Preallocation
            stateCombinations(size(id_combinations, 1), n_vehicles) = State([], [], [], []); 
            for id_vehicle = 1:n_vehicles
                stateCombinations(:, id_vehicle) = states(id_combinations(:, id_vehicle), ...
                                                          id_vehicle);
            end
        end
    end
end

