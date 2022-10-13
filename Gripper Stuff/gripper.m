clf
clc
clear all
hold on
axis equal

workspace = [-0.25 0.25 -0.25 0.25 -0 0.25];
hold on;
axis equal;
name1 = ['GripperL',datestr(now,'yyyymmddTHHMMSSFFF')];
name2 = ['GripperR',datestr(now,'yyyymmddTHHMMSSFFF')];

L(1) = Link([0      0      0.05      0      0]);
L(2) = Link([0      0      0.045      0      0]);
L(3) = Link([0      0      0.045       0      0]);

L(1).qlim = [-90 90]*pi/180;
L(2).qlim = [-90 90]*pi/180;
L(3).qlim = [-90 90]*pi/180;

L(1).offset = -16.1975*pi/180;
L(2).offset = 58*pi/180;
L(3).offset = 48*pi/180;

scale = 0.25;
q = zeros(1,3);
base = eye(4);

%PlaceObject('gripper.ply',[0,0,0])

GripperL = SerialLink(L,'name',name1,'base',base);
GripperR = SerialLink(L,'name',name2,'base',base);
GripperL.base = GripperL.base*trotx(pi/2);
GripperR.base = GripperR.base*trotz(pi)*trotx(pi/2);

%% texture L
for linkIndex = 0:GripperL.n
    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    GripperL.faces{linkIndex + 1} = faceData;
    GripperL.points{linkIndex + 1} = vertexData;
end

% Display robot
GripperL.plot3d(zeros(1,GripperL.n),'noarrow','workspace',workspace);
% view(3)
% drawnow()
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
GripperL.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:GripperL.n
    handles = findobj('Tag', GripperL.name);
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

GripperL.teach
%% texture R
for linkIndex = 0:GripperR.n
    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    GripperR.faces{linkIndex + 1} = faceData;
    GripperR.points{linkIndex + 1} = vertexData;
end

% Display robot
GripperR.plot3d(zeros(1,GripperR.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
GripperR.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:GripperR.n
    handles = findobj('Tag', GripperR.name);
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
%% Qmatrix for Gripper animation
qopen = zeros(1,3);
qclose = [-0.2513    0.6912   -0.4398];
qmatc = jtraj(qopen,qclose,100);
qmato = jtraj(qclose,qopen,100);


%% Close Gripper
for i = 1:100
    pause(0.01)
    GripperL.animate(qmatc(i,:));
    GripperR.animate(qmatc(i,:));
end

%% Open Gripper
for i = 1:100
    pause(0.01)
    GripperL.animate(qmato(i,:));
    GripperR.animate(qmato(i,:));
end
