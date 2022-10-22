ur3 = GetUR3;
% Set parameters
t = 5; % Total time to execute trajectory
deltaT = 0.02; % control frequency (Hz) 
steps = t/deltaT; % No. of steps for simulation which is based on t and deltaT 
epsilon = 0.1; % Unique, base on manipulator, play around with simulation to 
               % get the value. Threshold value for manipulability / Damped Least Squares 
W = diag([1 1 1 0.1 0.1 0.1]) % Weighting matrix for the velocity vector. Weighs how important x y z r p
                              % y is. Want to give more value to x y z?

                              
% Obtain initial data 
% Trasnformation of first point and angle
% Initial guess for joint angles (robot.getpos)
% The first waypoint of the qMatrix
% Trajectory calculations

% RMRC: use the redundant equation as there are 7 joints m < n. 