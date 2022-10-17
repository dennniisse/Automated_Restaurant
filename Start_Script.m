clear; clc; clf;
hold on;
dobot = GetDobot();
UR3 = GetUR3();
menu = Menu();

% Test to move UR3 to ingredients
UR3.move(menu.breadStorageLocation, false, menu);
