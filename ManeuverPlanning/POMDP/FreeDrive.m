classdef FreeDrive < Maneuver
    %FREEDRIVE Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function [speed_new, s_new, d_new, orientation_new] = apply(state,deltaT)
            %Apply some acceleration            
            speed_new = state.speed +1;
            s_new = state.s + speed_new*deltaT;
            d_new = state.d;
            orientation_new = state.orientation;
            %nextState = [speed_new, s_new, d_new, orientation_new];
        end
    end
end

