clear; clc; clf;
hold on;
dobot = GetDobot();
UR3 = GetUR3();
menu = Menu();

% Test to move UR3 to ingredients
goal = [0 0 0.6];
UR3.move(goal);
