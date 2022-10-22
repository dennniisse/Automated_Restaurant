%% Robotics
% Set parameters
% Total time to execute trajectory
 % control frequency (Hz) 
 % No. of steps for simulation which is based on t and deltaT 
 % Unique, base on manipulator, play around with simulation to 
               % get the value. Threshold value for manipulability / Damped Least Squares 
% Weighting matrix for the velocity vector. Weighs how important x y z r p
                              % y is. Want to give more value to x y z?

                              
% Obtain initial data 
% Trasnformation of first point and angle
% Initial guess for joint angles (robot.getpos)
% The first waypoint of the qMatrix
% Trajectory calculations

% RMRC: use the redundant equation as there are 7 joints m < n. 
clc
goal = transl([-1 0 0])*troty(pi);
h = PlaceObject("rock.ply",goal(1:3,4)');
goal = transl([-1 0 0.2])*troty(pi);
workspace = [-4 4 -4 4 0 3];
% 1.1) Set parameters for the simulation
self = GetUR3;        % Load robot model
t = 1;             % Total time (s)
deltaT = 0.02;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
% Allocate array data
qMatrix = zeros(steps,7);       % Array for joint anglesR
qdot = zeros(steps,7);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
% Get current pose
q0 = self.model.getpos;                   % Initial guess for joint angles
T1 = self.model.fkine(q0);% T1 get curernt Pose and make into homoegenous transform
x1 = [T1(1,4) T1(2,4) T1(3,4)];% x1 get the x y of T1
x2 = [goal(1,4) goal(2,4) goal(3,4)];% x2 get the x y of T2 (which is the goal x and y)
th1 = tr2rpy(T1);
th2 = tr2rpy(goal);

% Set up trajectory
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    x(:,i) = (1-s(i))*x1 + s(i)*x2; % Points in xyz
    theta(:,i) = (1-s(i))*th1 + s(i)*th2; % R P Y angles
    theta(3,i) = pi/2;
end
qMatrix(1,:) = self.model.ikcon(T1,q0);   % Solve joint angles to achieve first waypoint
% Track the trajectory with RMRC
for i = 1:steps-1
    T1 = self.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T1(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T1(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = self.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m = sqrt(det(J*J'));
    if m < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(7))*J'; % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:7                                                             % Loop through joints 1 to 7
        if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
end

axis equal

% 1.5) Plot the results
hold on;
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
for i = 1:steps
    self.model.animate(qMatrix(i,:));
    self.transformGripper(steps,true);
end
