classdef OthersInfo
    %OTHERSINFO keeps the info about vehicles other than ego
    %   Detailed explanation goes here
    
    properties
        vOtherVehicles_ref
        sigmaS
        sigmaV
    end
    
    methods
        function obj = OthersInfo(vOtherVehicles_ref, sigmaS, sigmaV)
            %OTHERSINFO Construct an instance of this class
            obj.vOtherVehicles_ref = vOtherVehicles_ref;
            obj.sigmaS = sigmaS;
            obj.sigmaV = sigmaV;
        end
        
    end
end

