clear; clc; clf;
hold on;
% dobot = GetDobot();
UR3 = GetUR3();
% self.brickLocation = [0 0 0];
% [f,self.v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% self.brickVertexCount = size(self.v,1);
% self.brick_h(1) = trisurf(f,self.v(:,1)+self.brickLocation(brickIndex,1)...
%     , self.v(:,2)+self.brickLocation(brickIndex,2)...
%     , self.v(:,3)+self.brickLocation(brickIndex,3)...
%     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Test to move UR3 to ingredients
% goal = [-2 0 0];
% UR3.move(goal, false);
