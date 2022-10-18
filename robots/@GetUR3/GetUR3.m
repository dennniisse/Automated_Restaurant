%% CHECKLIST
% Finish end effector and add the end effector to the move function
%%
classdef GetUR3 < handle
    properties
        model
        modelRight
        modelLeft
    end
    properties (Access = private)
        workspace = [-9 9 -9 9 0 6];
        steps = 15;
        % environment handles
        env_h; m_h;
        gripperOffset = 0.15
    end
    
    methods
        function self = GetUR3(self)
            self.GetRobot();
%             self.GetEnvironment();
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
            
            L(1).qlim = [-3 0];
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-360 360]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            L(7).qlim = [-360 360]*pi/180;
            
            self.model = SerialLink(L,'name',name);
            self.model.delay = 0;
            self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);
            
            for linkIndex = 1:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['rover_ur3_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            % Plot UR3 as 3D
            q = zeros(1,7);
            self.model.plot3d(q,'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
%             self.model.teach();
            % Colour UR3
            for linkIndex = 1:self.model.n
                handles = findobj('Tag', self.model.name); %findobj: find graphics objects with
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
            self.env_h(1) = surf([-8,-8;8,8],[-8,8;-8,8],[0,0;0,0],'CData',imread('ground_mars.jpg'),'FaceColor','texturemap');
            self.env_h(2) = surf([8,-8;8,-8],[8,8;8,8],[5,5;0,0],'CData',imread('wall_mars.jpg'),'FaceColor','texturemap');
            %             surf([8,8;8,8],[8,-8;8,-8],[5,5;0,0],'CData',imread('wall_mars_1.jpg'),'FaceColor','texturemap');
            self.m_h(1) = PlaceObject("BeachRockFree_decimated.ply",[-8 8 0]);
            self.m_h(2) = PlaceObject("BeachRockFree_decimated.ply",[-4 8 0]);
            self.m_h(3) = PlaceObject("BeachRockFree_decimated.ply",[0 8 0]);
            self.m_h(4) = PlaceObject("BeachRockFree_decimated.ply",[3.7 8 0]);
        end
        
        function RemoveEnvironment(self)
            delete(self.env_h);
            delete(self.m_h);
        end
        function GetGripper(self)
            gripperBase = self.model.fkine(self.model.getpos());
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
            gripperBase = self.model.fkine(self.model.getpos());
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
        
%         function move(self,goal,gripperBool) 
%             q2 = goal;
%             q2(3) = goal(3) + self.gripperOffset;
%             newQ = eye(4)*transl(q2)*troty(pi);
%             finalPos = self.model.ikcon(newQ);
%             intPos = self.model.getpos();
%             s = lspb(0,1,self.steps);
%             qMatrix = nan(self.steps,self.model.n);
%             for i = 1:self.steps
%                 qMatrix(i,:) = (1-s(i))*intPos + s(i)*finalPos;
%                 self.model.animate(qMatrix(i,:));
%                 self.transformGripper(self.steps,gripperBool);
%                 drawnow();                
%             end
%         end
        
        function move(self,goal,gripperBool) 
            % steps
            T1 = self.model.fkine(self.model.getpos);% T1 get curernt Pose and make into homoegenous transform
            x1 = [T1(1,4) T1(2,4)];% x1 get the x y of T1
            x2 = [goal(1) goal(2)];% x2 get the x y of T2 (which is the goal x and y)
            deltaT = 0.05; % discrete time step
            x = zeros(2,self.steps);
            s = lspb(0,1,self.steps);                                 % Create interpolation scalar
            for i = 1:self.steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
            end
            qMatrix = nan(self.steps,self.model.n);
            qMatrix(1,:) = self.model.ikcon(transl(goal));
            for i = 1:self.steps
                xdot = (x(:,i+1) - x(:,i))/deltaT;  
                J = self.model.jacob0(qMatrix(i,:));
                J = J(1:6,1:6);
                qdot = inv(J)*xdot
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';
                
                self.model.animate(qMatrix(i,:));
                self.transformGripper(self.self.steps,gripperBool);
                drawnow();  
            end
            
        end
        
%         function selfCollision
%             % select specified joint angles (the more the better) 
%         end
    end
end

