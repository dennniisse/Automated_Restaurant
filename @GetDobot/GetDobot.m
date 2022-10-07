%% CHECKLIST
% Finish end effector and add the end effector to the move function
% Dobot movement function
%%
classdef GetDobot < handle
    properties
        dobot
    end
    properties (Access = private)
        workspace = [-3 3 -3 3 -0.75 6];
    end
    
    methods
        function self = GetDobot(self)
            self.GetRobot();
        end
        
        function GetRobot(self)
            name = 'dobot';
            % dh = [THETA D A ALPHA SIGMA OFFSET]
            L(1) = Link([0    0.1395    0       pi/2   0]);
            L(2) = Link([0    0.1330  0.2738     0     0]);
            L(3) = Link([0    -0.1165  0.230      0     0]);
            L(4) = Link([0      0.116   0      pi/2    0]);
            L(5) = Link([0      0.116   0     -pi/2  	 0]);
            L(6) = Link([0      0       0       0      0]);
            
            self.dobot = SerialLink(L,'name',name);
            self.dobot.delay = 0;
            
            L(1).qlim = [-360 360]*pi/180;
            L(2).qlim = [-90 90]*pi/180;
            L(3).qlim = [-170 170]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(2).offset = pi/2;
            L(4).offset = pi/2;
            
            for linkIndex = 0:self.dobot.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['dobotlink_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.dobot.faces{linkIndex + 1} = faceData;
                self.dobot.points{linkIndex+1} = vertexData;
            end
            % Plot dobot as 3D
            self.dobot.plot3d((zeros(1,self.dobot.n)),'workspace',self.workspace);
            
            % Colour dobot
            for linkIndex = 0:self.dobot.n
                handles = findobj('Tag', self.dobot.name); %findobj: find graphics objects with
                %specific properties
                %'Tag': a property name, therefore
                %it's finding objects whose Tag is
                %dobot
                %h will return the all objects in
                %the hierarchy that have their Tag
                %property set to value 'dobot'
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
            
        end
        
        function getGripper(self)
            
        end
        
        function moveDobot(self)
            finalPos = goal;
            intPos = self.dobot.getpos();
            s = lspb(0,1,self.steps);
            qMatrix = nan(self.steps,7);
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
                self.dobot.animate(qMatrix(i,:));
                drawnow();
            end
        end
    end
end