clear; clc; clf;
hold on;
% dobot = GetDobot();
UR3 = GetUR3();
dobot = GetDobot();
steps = 50; 
% Test to move UR3 to ingredients
goal = [-1 0 0]; % goal = [-0.20 0.75 0];
h = PlaceObject("rock.ply",goal);
qMatrixUR3 = UR3.GetQMatrix(goal, true); 
qMatrixDobot = dobot.GetQMatrix(goal,true);

for i = 1 : size(qMatrixUR3,1)
    UR3.model.animate(qMatrixUR3(i,:));
    dobot.model.animate(qMatrixDobot(i,:));
    UR3.transformGripper(steps,true); 
    dobot.transformGripper(steps, false); 
end
% goal = [-0.112 0.539 0.298];
% UR3.initDropOff();
% UR3.move(goal,false);

