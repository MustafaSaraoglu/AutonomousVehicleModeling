classdef Values
% Compare values according to safety and liveness
    
    properties
        safety % safety value
        liveness % liveness value
    end
    
    methods
        function obj = Values(safety, liveness)
            %VALUES Construct an instance of this class
            obj.safety = safety;
            obj.liveness = liveness;
        end
    end
    
    methods(Static)
        function isMet = isGreater(value1, value2)
        % Check if value1 is greater than value2
            
            isMet = false;
            if (value1.safety > value2.safety) || ...
                    (value1.safety == value2.safety && value1.liveness > value2.liveness)
                isMet = true;
                return
            end
        end
        
        function isMet = isLess(value1, value2)
        % Check if value1 is less than value2
            
            isMet = false;
            if (value1.safety < value2.safety) || ...
                    (value1.safety == value2.safety && value1.liveness < value2.liveness)
                isMet = true;
                return
            end
        end
        
         function isMet = isLessEqual(value1, value2)
        % Check if value1 is less than or equal to value2
            
            isMet = false;
            if (value1.safety < value2.safety) || ...
                    (value1.safety == value2.safety && value1.liveness <= value2.liveness)
                isMet = true;
                return
            end
         end
    
        function maxValue = Max(value1, value2)
        % Find maximum between two values
            
            maxValue = value2;
            if Values.isGreater(value1, value2)
                maxValue = value1;
            end
        end
        
        function minValue = Min(value1, value2)
        % Find minimum between two values
            
            minValue = value2;
            if Values.isLess(value1, value2)
                minValue = value1;
            end
        end
    end
end

