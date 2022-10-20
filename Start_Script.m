clear; clc; clf;
hold on;
% dobot = GetDobot();
UR3 = GetUR3();

% Test to move UR3 to ingredients
% goal = [-1 0 0];
% h = PlaceObject("rock.ply",goal);
% UR3.move(goal, false);

GUIAPP = GUI;
GUIAPP.getRobot(UR3)

while(1)
    controlm = GUIAPP.read;
end