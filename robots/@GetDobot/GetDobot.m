%% CHECKLIST
% Finish end effector and add the end effector to the move function
% Dobot movement function
%%
classdef GetDobot < handle
    properties
        model
    end
    properties (Access = private)
        workspace = [-3 3 -3 3 -0.75 6];
        steps = 50;
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
            L(6) = Link([0      0      0       0      0]);
            
            self.model = SerialLink(L,'name',name);
            self.model.base = self.model.base * transl([0.8 0 0]);

            self.model.delay = 0;
            
            L(1).qlim = [-360 360]*pi/180;
            L(2).qlim = [-90 90]*pi/180;
            L(3).qlim = [-170 170]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(2).offset = pi/2;
            L(4).offset = pi/2;
            
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['dobotlink_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            % Plot dobot as 3D
            self.model.plot3d((zeros(1,self.model.n)),'workspace',self.workspace);
%             self.model.teach();
            
            % Colour dobot
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name); %findobj: find graphics objects with
                h = get(handles,'UserData');        

                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...                                        
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            hold on;
        end
        
        function getGripper(self)
            
        end

        function move(self,goal,moveOthersFlag,itemHandle) % if true, then we move the handle
            newQ = eye(4)*transl(goal)*troty(pi);
            finalPos = self.model.ikcon(newQ);
            intPos = self.model.getpos();
            s = lspb(0,1,self.steps);
            qMatrix = nan(self.steps,self.model.n);
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
                self.model.animate(qMatrix(i,:));
                drawnow(); 
                if moveOthersFlag == true;
                    eeBase = self.model.fkine(self.model.getpos());
                    itemHandle.moveTray(eeBase);
                end
            end
        end
        
        function resetPose(self)
            finalPos = zeros(1,6);
            intPos = self.model.getpos();
            s = lspb(0,1,self.steps);
            qMatrix = nan(self.steps,self.model.n);
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
                self.model.animate(qMatrix(i,:));
                drawnow();
            end
        end
    end
end