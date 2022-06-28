classdef LaneChanging < Maneuver
    %LaneChanging Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function nextState = apply(state,deltaT)
            %Apply Lane Changing
            speed_new = state.speed;
            s_new = state.s + speed_new*deltaT;
            if abs(state.d)<0.05 % tolerance for d value
                d_new = 3.7;
            elseif abs(state.d)-3.7<0.05
                d_new = 0;
            end
            orientation_new = state.orientation;
            
            nextState = StateV(s_new,d_new,orientation_new,speed_new);
        end
    end
end


