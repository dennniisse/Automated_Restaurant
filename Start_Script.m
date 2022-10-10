clear; clc; clf;
hold on;
dobot = GetDobot();
% UR3 = GetUR3();
menu = Menu();

% Test for movement function 
dobot.move(menu.trayStorageLocation, false, menu);
dobot.move(menu.trayOrderLocation, true, menu); 
dobot.resetPose();