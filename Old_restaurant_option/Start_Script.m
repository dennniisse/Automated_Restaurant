clear; clc; clf;
hold on;
dobot = GetDobot();
UR3 = GetUR3();
menu = Menu();

% Test for movement function, this moves the CR3 to get and drop the tray
dobot.move(menu.trayStorageLocation, false, menu);
dobot.move(menu.trayOrderLocation, true, menu); 
dobot.resetPose();

% Test to move UR3 to ingredients
UR3.move(menu.breadStorageLocation, false, menu);
UR3.move(menu.trayOrderLocation, true, menu);