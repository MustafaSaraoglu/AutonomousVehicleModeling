classdef State
    %STATE Summary of this class goes here
    
    properties
        s
        d
        orientation
        speed
    end
    
    methods
        function obj = State(s,d,orientation,speed)
            %STATE Construct an instance of this class
            obj.s = s;
            obj.d = d;
            obj.orientation = orientation;
            obj.speed = speed;
        end
    end
end

