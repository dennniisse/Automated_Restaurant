classdef GetDobot < handle
    properties
        % bread: white, wholemeal
        % meat: chicken, beef
        % vegetables: lettuce, tomato, corn
        % dairy: cheese, butter
        % sauce: tomato, mustard, bbq
        
    end
    properties (Access = private)
        workspace = [-3 3 -3 3 -0.75 6];
    end
    
    methods
        function self = GetDobot(self)
            self.GetRobot();
        end
        
        function GetMenu(self)
            
        end
        
        
        function (self)
        end
    end
end