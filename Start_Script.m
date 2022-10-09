clear; clc; clf;
figure;
hold on;
dobot = GetDobot();
UR3 = GetUR3();

offset = -0.67; %adjusting the height of environment so UR3 is placed on top of the table
[f,v,data] = plyread('tray.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% Plot the environment, account for any changes in the UR3's base
trisurf(f,v(:,1) + 0.75...
    , v(:,2) + 0.4 ...
    , v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
