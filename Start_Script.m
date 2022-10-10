clear; clc; clf;
hold on;
dobot = GetDobot();
UR3 = GetUR3();
menu = Menu();

% Test for movement function
dobot.move(menu.trayLocation);
