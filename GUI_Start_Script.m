clear; clc; clf;
hold on;
axis equal;
UR3 = GetUR3();

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