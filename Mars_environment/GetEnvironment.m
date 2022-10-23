classdef GetEnvironment < handle
    properties
        payload_h = [patch patch patch patch patch patch patch patch patch]; % must initialise as a patch
        payloadLocation;
        hopperLocation = [-0.5 0.45 0.4];
    end
    
    properties (Access = private)
        imgSize = 12;
        env_h; rocks_h; % environment handles
        count = 0; % keeps count of environment handle
        payload_rocksVertexCount; v_rocks; % to move payload
        payload_tilesVertexCount; v_tiles;
        placeTile = [3.3 -0.133 0]; % initialise
    end
    
    methods
        function self = GetEnvironment
            count = 0;
        end
        function LoadEnvironment(self)
            self.env_h(1) = surf([-self.imgSize,-self.imgSize;self.imgSize,self.imgSize],[-self.imgSize,self.imgSize;-self.imgSize,self.imgSize],[0,0;0,0],'CData',imread('ground_mars.jpg'),'FaceColor','texturemap');
            self.env_h(2) = surf([self.imgSize,-self.imgSize;self.imgSize,-self.imgSize],[self.imgSize,self.imgSize;self.imgSize,self.imgSize],[5,5;0,0],'CData',imread('wall_mars.jpg'),'FaceColor','texturemap');
            self.env_h(3) = surf([self.imgSize,self.imgSize;self.imgSize,self.imgSize],[self.imgSize,-self.imgSize;self.imgSize,-self.imgSize],[5,5;0,0],'CData',imread('wall_mars_1.jpg'),'FaceColor','texturemap');
            self.env_h(4) = PlaceObject("spacebase.ply", [7.8 7 0]);
            self.env_h(5) = PlaceObject("crate.ply",[0 0 0]);
            self.rocks_h(1) = PlaceObject("BeachRockFree_decimated.ply",[-self.imgSize self.imgSize 0]);
            self.rocks_h(2) = PlaceObject("BeachRockFree_decimated.ply",[-(self.imgSize-4) self.imgSize 0]);
            self.rocks_h(5) = PlaceObject("rockypath.ply",[0 0 0]);
            hopper = PlaceObject("hopper.ply",[-0.5 0.45 0]);
        end
        
        
        function RemoveEnvironment(self)
            delete(self.env_h);
            delete(self.rocks_h);
        end
        
        function [index] = GetRock(self,base)
            base = base;
            self.count = self.count + 1; % this is used as the index for the payload_h
            index = self.count;           
            % upload into environment
            % Obtain ply data
            [f,self.v_rocks,data] = plyread('rock.ply','tri'); 
            % scale vertex colour
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Get payload vertices count, will be used  to transform payload location
            self.payload_rocksVertexCount= size(self.v_rocks,1); %Obtain row size only
            self.payload_h(self.count) = trisurf(f,self.v_rocks(:,1)+base(1)...
                , self.v_rocks(:,2)+base(2)...
                , self.v_rocks(:,3)+base(3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            % store location
            self.payloadLocation(self.count, 1) = base(1);
            self.payloadLocation(self.count, 2) = base(2);
            self.payloadLocation(self.count, 3) = base(3);
            
        end
        
        function [index] = GetTiles(self,base)
            self.count = self.count + 1;
            index = self.count;
            [f,self.v_tiles,data] = plyread('tile.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            self.payload_tilesVertexCount = size(self.v_tiles,1);
            self.payload_h(self.count) = trisurf(f,self.v_tiles(:,1)+base(1)...
                , self.v_tiles(:,2)+base(2)...
                , self.v_tiles(:,3)+base(3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            self.payloadLocation(self.count, 1) = base(1);
            self.payloadLocation(self.count, 2) = base(2);
            self.payloadLocation(self.count, 3) = base(3);
        end 
        
        %         GetRocks and GetTiles requires a base input of array 1 x 3 [x y z]
        
        %         UpdateLocation requires a base input of array 4 x 4, homogenous transformation
                
        function UpdateLocation(self,index,eeBase,payloadtype) % payloadtype = 'rock' or 'tile'
            % update location in array            
            self.payloadLocation(index, 1) = eeBase(1,4);
            self.payloadLocation(index, 2) = eeBase(2,4);
            self.payloadLocation(index, 3) = eeBase(3,4);
            % update ply file data    
            switch payloadtype
                case 'rock'
                    newLocation = [eeBase * [self.v_rocks,ones(self.payload_rocksVertexCount,1)]']';                  
                case 'tile'                    
                    newLocation = [eeBase * [self.v_tiles,ones(self.payload_tilesVertexCount,1)]']';
            end
            % update location in matlab 
            self.payload_h(index).Vertices = newLocation(:,1:3);
        end
        
        function [index] = LoadTiles(self)
            % w = 0.133 ; h = 0.034
            index = [];
            for y = -0.133:0.17:0.133 
                for z = (0.034*7):-0.034:0
                    base = [2.7 y  z]; ind = self.GetTiles(base);
                    self.payloadLocation(ind, 1) = base(1);
                    self.payloadLocation(ind, 2) = base(2);
                    self.payloadLocation(ind, 3) = base(3);
                    index = [index ; ind];
                end
            end   
        end 
        
        function [location] = getTileLocation(self,index)
            if(index < 4)
                location = self.placeTile;
                self.placeTile(2) = self.placeTile(2) + 0.133; % increase by width of tile 
            elseif (index > 3)  
                self.placeTile(1) = 3.2300;
                location = self.placeTile; 
                self.placeTile(2) = self.placeTile(2) - 0.133;                
            end 
                
        end 
        
        
    end
    
end