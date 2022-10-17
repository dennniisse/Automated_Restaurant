%% CHECKLIST
% Finish end effector and add the end effector to the move function
%%
classdef GetUR3 < handle
    properties
        UR3
        modelRight
        modelLeft
    end
    properties (Access = private)
        workspace = [-9 9 -9 9 -0.1 6];
        steps = 15;
    end
    
    methods
        function self = GetUR3(self)
            self.GetRobot();
            self.GetEnvironment();
            self.GetGripper();
        end
        
        function GetRobot(self)
            name = 'UR3';
            % dh = [THETA D A ALPHA SIGMA OFFSET]
            L(1) = Link([pi    0      0   pi/2    1   0]);
            L(2) = Link([0    0.383  0   pi/2    0   0]); %.1519 + .5174
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
            
            for linkIndex = 1:self.UR3.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['rover_ur3_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
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
            for linkIndex = 1:self.UR3.n
                handles = findobj('Tag', self.UR3.name); %findobj: find graphics objects with
                h = get(handles,'UserData');
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
        
        
        function GetEnvironment(self)
            surf([-8,-8;8,8],[-8,8;-8,8],[0,0;0,0],'CData',imread('ground_mars.jpg'),'FaceColor','texturemap');
            surf([8,-8;8,-8],[8,8;8,8],[5,5;0,0],'CData',imread('wall_mars.jpg'),'FaceColor','texturemap');
            %             surf([8,8;8,8],[8,-8;8,-8],[5,5;0,0],'CData',imread('wall_mars_1.jpg'),'FaceColor','texturemap');
            
        end
        
        function GetGripper(self)
            gripperBase = self.UR3.fkine(self.UR3.getpos());
            L(1) = Link([0 0 0 0 1 0]);
            L(2) = Link([0   0    0.01   0   0   0]);
            L(1).qlim = [0 0]*pi/180; %make base static
            L(2).qlim = [-5 10]*pi/180; %trial and error to figure out the limit using .teach()
            self.modelRight = SerialLink(L,'name','gripperRight');
            self.modelRight.delay = 0;
            self.modelRight.base =  gripperBase * troty(pi/2);%self.modelRight.base * transl([[gripperBase(1), gripperBase(2), gripperBase(3)]]);
            
            %Plot annd Colour Gripper
            for linkIndex = 1:self.modelRight.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['gripper_ur3_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.modelRight.faces{linkIndex + 1} = faceData;
                self.modelRight.points{linkIndex + 1} = vertexData;
            end
            q = [0,-5]*pi/180; % gripper open as wide as possible
            self.modelRight.plot3d(q,'workspace',self.workspace);
            %             self.modelRight.teach();
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            
            for linkIndex = 0:1
                handles = findobj('Tag', self.modelRight.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    continue;
                end
            end
            
            L = Link([0   0    0.01   0   0   0]);
            L.qlim = [-10 5]*pi/180;
            self.modelLeft = SerialLink(L,'name','gripperLeft');
            self.modelLeft.delay = 0;
            self.modelLeft.base = gripperBase* troty(pi/2); %self.modelLeft.base * transl([[gripperBase]]);
            
            % Plot Left Finger 
            [ faceData, vertexData, plyData{2} ] = plyread(['gripper_ur3_3.ply'],'tri'); %#ok<AGROW>
            self.modelLeft.faces{2} = faceData;
            self.modelLeft.points{2} = vertexData;
            self.modelLeft.plot3d(5*pi/180,'workspace',self.workspace,'arrow');
            
            % Colour Left Finger   
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end                     
            handles = findobj('Tag', self.modelLeft.name); 
            h = get(handles,'UserData');
            try
                h.link(2).Children.FaceVertexCData = [plyData{2}.vertex.red ...
                    , plyData{2}.vertex.green ...                                  
                    , plyData{2}.vertex.blue]/255;
                h.link(2).Children.FaceColor = 'interp';
            catch ME_1
            end
        end
        
        function OpenGripper(self)
            rightQ = [0,-5]*pi/180;
            leftQ = 5*pi/180;
            self.modelRight.animate([0,rightQ(2)]); %move gripper
            self.modelLeft.animate(leftQ); %move gripper 
        end 
        
        function transformGripper(self,steps,gripperClose) %true close gripper
            %transform Base
            gripperBase = self.UR3.fkine(self.UR3.getpos());
            if (gripperClose == true)
            q = deg2rad(10/steps); % the distance the gripper moves is already set (15deg). 
                                        % Therefore, using steps the UR3 is moving from, the distance at each step can be computer
            rightQ = self.modelRight.getpos() + q ; %return q = [q1 q2], move towards +ve 5deg to close
            leftQ = self.modelLeft.getpos() - q; %return q = [q1], move towards -ve 10deg to close            
            self.modelRight.base = gripperBase* troty(pi/2)* trotx(pi/2);
            self.modelLeft.base = gripperBase* troty(pi/2)* trotx(pi/2);            
            %Move gripper            
                self.modelRight.animate([0,rightQ(2)]); %move gripper
                self.modelLeft.animate(leftQ); %move gripper
            end
            if (gripperClose == false)
                rightQ = self.modelRight.getpos();
                leftQ = self.modelLeft.getpos();
                self.modelRight.base = gripperBase* troty(pi/2)* trotx(pi/2);
                self.modelLeft.base = gripperBase* troty(pi/2)* trotx(pi/2);
                self.modelRight.animate([0,rightQ(2)]); %move gripper
                self.modelLeft.animate(leftQ); %move gripper                
            end
        end
        
        function move(self,goal) 
            newQ = eye(4)*transl(goal)*troty(pi);
            finalPos = self.UR3.ikcon(newQ);
            intPos = self.UR3.getpos();
            s = lspb(0,1,self.steps);
            qMatrix = nan(self.steps,self.UR3.n);
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
                self.UR3.animate(qMatrix(i,:));
                self.transformGripper(self.steps,false);
                drawnow();                
            end
        end
        
    end
end

