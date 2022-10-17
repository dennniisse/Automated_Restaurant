clear all
name = ['DobotCR3',datestr(now,'yyyymmddTHHMMSSFFF')];
% L(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
L(1) = Link([0    0.1395    0       pi/2   0]);
L(2) = Link([0    0.1330  0.2738     0     0]);
L(3) = Link([0    -0.1165  0.230      0     0]);
L(4) = Link([0      0.116   0      pi/2    0]);
L(5) = Link([0      0.116   0     -pi/2  	 0]);
L(6) = Link([0      0       0       0      0]);

DobotCR3 = SerialLink(L,'name',name);
% q = zeros(1,6);
% scale = 0.5;
workspace = [-1 1 -1 1 -1 1];

L(1).qlim = [-360 360]*pi/180;
L(2).qlim = [-90 90]*pi/180;
L(3).qlim = [-170 170]*pi/180;
L(4).qlim = [-360 360]*pi/180;
L(5).qlim = [-360 360]*pi/180;
L(6).qlim = [-360 360]*pi/180;
L(2).offset = pi/2;
L(4).offset = pi/2;
%DobotCR3.plot(q,'scale',scale)
%DobotCR3.teach

for linkIndex = 0:DobotCR3.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    DobotCR3.faces{linkIndex+1} = faceData;
    DobotCR3.points{linkIndex+1} = vertexData;
end

% Display robot
DobotCR3.plot3d(zeros(1,DobotCR3.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
DobotCR3.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:DobotCR3.n
    handles = findobj('Tag', DobotCR3.name);
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
DobotCR3.teach