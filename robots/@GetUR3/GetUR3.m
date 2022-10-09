%% CHECKLIST
% Finish end effector and add the end effector to the move function 
%%
classdef GetUR3 < handle
    properties
        UR3
    end
    properties (Access = private)
        workspace = [-3 3 -3 3 -0.75 6];
%         gripperOfset = ;
    end
    
    methods
        function self = GetUR3(self)
            self.GetRobot();
        end
        
        function GetRobot(self)
            name = 'UR3';
            % dh = [THETA D A ALPHA SIGMA OFFSET]
            L(1) = Link([pi    0      0   pi/2    1   0]);
            L(2) = Link([0    0.1519  0   pi/2    0   0]);
            L(3) = Link([0    0   -0.24365   0    0   0]);
            L(4) = Link([0    0   -0.21325   0    0   0]);
            L(5) = Link([0    0.11235     0   pi/2    0   0]);
            L(6) = Link([0    0.08535     0   -pi/2    0   0]);
            L(7) = Link([0    0.0819      0   0   0   0]);
            
            L(1).qlim = [-0.8 0];
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-360 360]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(7).qlim = [-360 360]*pi/180;
            
            self.UR3 = SerialLink(L,'name',name);
            self.UR3.delay = 0;
            self.UR3.base = self.UR3.base * trotx(pi/2) * troty(pi/2);
            
            for linkIndex = 0:self.UR3.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['ur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.UR3.faces{linkIndex + 1} = faceData;
                self.UR3.points{linkIndex + 1} = vertexData;
            end
            % Plot UR3 as 3D
            q = zeros(1,7);
            self.UR3.plot3d(q,'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            % Colour UR3
            for linkIndex = 0:self.UR3.n
                handles = findobj('Tag', self.UR3.name); %findobj: find graphics objects with
                %specific properties
                %'Tag': a property name, therefore
                %it's finding objects whose Tag is
                %UR3
                %h will return the all objects in
                %the hierarchy that have their Tag
                %property set to value 'UR3'
                h = get(handles,'UserData');        %get: returns the value for 'UserData'.
                %h is a structure (see OneNote or
                %print onto cmd)
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ... %%as h is a structure we access h.link and iterate
                        , plyData{linkIndex+1}.vertex.green ...                                         %%through each link and obtain its colour
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            hold on;
        end
        
        function GetGripper(self)  
            
        end
        
        function moveUR3(self,goal) % goal = location of food / place
            finalPos = goal; 
            intPos = self.UR3.getpos(); 
            s = lspb(0,1,self.steps);
            qMatrix = nan(self.steps,7);
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
                self.UR3.animate(qMatrix(i,:));                
                drawnow();
            end
        end
        
    end
end

