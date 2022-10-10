classdef Menu < handle
    properties
        % bread: white, wholemeal
        % meat: chicken, beef
        % vegetables: lettuce, tomato, corn
        % dairy: cheese, butter
        % sauce: tomato, mustard, bbq
        trayLocation = [0.75 0.5 0]; % for some reason model can't reach it when it's at 0.8, no matter what y value I give it 
    end
    properties (Access = private)
        workspace = [-3 3 -3 3 -0.75 6];
        % create a key-value pair, name of food and XYZ coordinate
    end
    
    methods
        % constructor, stores the location of all the food menu;
        function self = Menu(self)
            self.getPlates();            
            
        end
        
        %Place the plates down
        function getPlates(self)
            offset = -0.67; %adjusting the height of environment so UR3 is placed on top of the table
            [f,v,data] = plyread('tray.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Plot the environment, account for any changes in the UR3's base
            trisurf(f,v(:,1) + self.trayLocation(1)...
                , v(:,2) + self.trayLocation(2) ...
                , v(:,3) + self.trayLocation(3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');     
        end 
        
        
        %
        function getMenuLocation(self)
            
        end 
        git 
        function GetMenu(self)
            %location of plates must be confirmed by checking coordinates
            %on blender model 
            
            
            
        end
        
        
        
    end
end
