clear; clc; clf;
hold on;
e = GetEnvironment();
e.LoadEnvironment; 
dobot = GetDobot();
% UR3 = GetUR3();

steps = 50; 
% % Test to move UR3 to ingredients
% goal = [-1 0 0]; % goal = [-0.20 0.75 0];
% rock(1) = e.GetRock(goal);
% qMatrixUR3 = UR3.GetQMatrix(goal, true); 
% qMatrixDobot = dobot.GetQMatrix(goal,true);

% for i = 1 : size(qMatrixUR3,1)
%     UR3.model.animate(qMatrixUR3(i,:));
%     dobot.model.animate(qMatrixDobot(i,:));
%     UR3.transformGripper(steps,true); 
%     dobot.transformGripper(steps, false); 
% end
% 
% goal = [0.3 0 0];
% qMatrixUR3 = UR3.GetQMatrix(goal, true); 
% 
% for i = 1 : size(qMatrixUR3,1)
%     UR3.model.animate(qMatrixUR3(i,:));
%     UR3.transformGripper(steps,true); 
%     ee = UR3.GeteeBase;
%     e.UpdateLocation(1,ee);
% end



