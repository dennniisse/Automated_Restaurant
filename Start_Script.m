clear; clc; clf;
hold on;
% dobot = GetDobot();
UR3 = GetUR3();

% Test to move UR3 to ingredients
goal = [-1 0 0];
h = PlaceObject("rock.ply",goal);
UR3.move(goal, false);
goal = [-0.112 0.539 0.298];
UR3.initDropOff();
UR3.move(goal,false);

