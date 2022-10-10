classdef Menu < handle
    properties
        % bread: white, wholemeal
        % meat: chicken, beef
        % vegetables: lettuce, tomato, corn
        % dairy: cheese, butter
        % sauce: tomato, mustard, bbq
        trayStorageLocation = [0.75 0.5 0]; % for some reason model can't reach it when it's at 0.8, no matter what y value I give it 
        trayOrderLocation = [0.3 0 0];% location where to place tray in front of UR3 
    end

    properties (Access = private)
        workspace = [-3 3 -3 3 -0.75 6];
        % create a key-value pair, name of food and XYZ coordinate
        tray_h;
        v;
        trayVertexCount;
    end
    
    methods
        % constructor, stores the location of all the food menu;
        function self = Menu(self)
            self.getPlates();            
            
        end
        
        %Place the plates down
        function getPlates(self)
            offset = -0.67; %adjusting the height of environment so UR3 is placed on top of the table
            [f,self.v,data] = plyread('tray.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.trayVertexCount = size(self.v,1);
            % Plot the environment, account for any changes in the UR3's base
            self.tray_h = trisurf(f,self.v(:,1) + self.trayStorageLocation(1)...
                , self.v(:,2) + self.trayStorageLocation(2) ...
                , self.v(:,3) + self.trayStorageLocation(3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');     
        end 
        
        function moveTray(self,eeBase)
            gripperOffset = 0;
            newLocation = [eeBase * troty(-pi) * transl([0 0 gripperOffset]) * [self.v,ones(self.trayVertexCount,1)]']';
            self.tray_h.Vertices = newLocation(:,1:3);
        end      
        
        
        %
        function getMenuLocation(self)
            
        end 
         
        function GetMenu(self)
            %location of plates must be confirmed by checking coordinates
            %on blender model 
            
            
            
        end
        
        
        
    end
end
