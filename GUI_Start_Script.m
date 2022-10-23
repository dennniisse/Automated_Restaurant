clear; clc; clf;
hold on;
axis equal;
UR3 = GetUR3();
dobot = GetDobot();
dobot.model.base = transl(1,0,0);
GUIAPP = GUI;
GUIAPP.getRobot(UR3,dobot)

while(1)
    [q1,q2] = GUIAPP.animateQvalues;
    UR3.model.animate(q1);
%     UR3.transformGripper(1,false);
    dobot.model.animate(q2);
%     dobot.transformGripper(1,false);
    drawnow();
end