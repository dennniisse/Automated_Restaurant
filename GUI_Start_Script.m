clear; clc; clf;
hold on;
axis equal;
% dobot = GetDobot();
UR3 = GetUR3();

% Test to move UR3 to ingredients
% goal = [-1 0 0];
% h = PlaceObject("rock.ply",goal);
% UR3.move(goal, false);

GUIAPP = GUI;
GUIAPP.getRobot(UR3)

while(1)
    q = GUIAPP.read;
    UR3.model.animate(q);
    UR3.transformGripper(1,false);
    GUIAPP
    pause(0.1);
    drawnow();
end