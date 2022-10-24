% index:
% get tiles current location
% get tiles next location 
% upload rocks, get location 

%%%%% start
% goal = goal to payload
% animate ur3 and cr3 going towards their payload, gripprs closing 
% change goal = new Location
% animate ur3 and cr3, with grippers still (close)
% open grippers
% repeat 
clear a; clear; clc; clf; 
% sensors intialisation 
a = arduino();

hold on; 
% hopper = PlaceObject("hopper.ply",[-0.5 0.45 0]);
e = GetEnvironment(); e.LoadEnvironment(); 
dobot = GetDobot();
f = msgbox("Press OK to start"); waitfor(f);
towerLight(a,'green',1);
UR3 = GetUR3();
steps = 50; 
tileCounter = 1;
dobotOffset = 0.30;

% dobot
tileNum = e.LoadTiles(); 
goalDobot = e.payloadLocation(tileCounter,:);
qMatrixDobot = dobot.GetQMatrix(goalDobot);

% ur3
goalUR3 = [-1.5 0 0]; 
rock(1) = e.GetRock(goalUR3);
qMatrixUR3 = UR3.GetQMatrix(goalUR3);
% animate
for i = 1 : size(qMatrixUR3,1) % use ur3 due to its larger size 
    eStop(a)
    dobot.model.animate(qMatrixDobot(i,:));
    dobot.transformGripper(steps,true);
    eStop(a)
    UR3.model.animate(qMatrixUR3(i,:));
    UR3.transformGripper(steps,true);
end

% % dobot
goalDobot = e.getTileLocation(tileCounter);
qMatrixDobot = dobot.GetQMatrix(goalDobot);

% ur3
goalUR3 = e.hopperLocation;
qMatrixUR3 = UR3.GetQMatrix(goalUR3);
for i = 1 : size(qMatrixDobot,1) 
    eStop(a)
    dobot.model.animate(qMatrixDobot(i,:));
    dobot.transformGripper(steps,false);  
    ee = dobot.GeteeBase;
    e.UpdateLocation(tileCounter,ee,'tile');    
    eStop(a)
    UR3.model.animate(qMatrixUR3(i,:));
    UR3.transformGripper(steps,false);
    ee = UR3.GeteeBase;
    e.UpdateLocation(rock(1),ee,'rock');
end
e.UpdateLocation(tileCounter,transl([goalDobot(1) goalDobot(2) 0]),'tile');
e.UpdateLocation(rock(1),transl([e.hopperLocation(1) e.hopperLocation(2) 0]),'rock');
tileCounter = tileCounter + 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dobot
tileNum = e.LoadTiles(); 
goalDobot = e.payloadLocation(tileCounter,:);
qMatrixDobot = dobot.GetQMatrix(goalDobot);

% ur3
goalUR3 = [-3.5 -0.1 0]; 
rock(1) = e.GetRock(goalUR3);
qMatrixUR3 = UR3.GetQMatrix(goalUR3);
% animate
for i = 1 : size(qMatrixUR3,1) % use ur3 due to its larger size 
    eStop(a)
    dobot.model.animate(qMatrixDobot(i,:));
    dobot.transformGripper(steps,true);
    eStop(a)
    UR3.model.animate(qMatrixUR3(i,:));
    UR3.transformGripper(steps,true);
end

% % dobot
goalDobot = e.getTileLocation(tileCounter);
qMatrixDobot = dobot.GetQMatrix(goalDobot);

% ur3
goalUR3 = e.hopperLocation;
qMatrixUR3 = UR3.GetQMatrix(goalUR3);
for i = 1 : size(qMatrixDobot,1) 
    eStop(a)
    dobot.model.animate(qMatrixDobot(i,:));
    dobot.transformGripper(steps,false);  
    eStop(a)
    ee = dobot.GeteeBase;
    e.UpdateLocation(tileCounter,ee,'tile');    
    eStop(a)
    UR3.model.animate(qMatrixUR3(i,:));
    UR3.transformGripper(steps,false);
    eStop(a)
    ee = UR3.GeteeBase;
    e.UpdateLocation(rock(1),ee,'rock');
end
e.UpdateLocation(tileCounter,transl([goalDobot(1) goalDobot(2) 0]),'tile');
e.UpdateLocation(rock(1),transl([e.hopperLocation(1) e.hopperLocation(2) 0]),'rock');
tileCounter = tileCounter + 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dobot
e.RemoveEnvironment;
tileNum = e.LoadTiles(); 
goalDobot = e.payloadLocation(tileCounter,:);
qMatrixDobot = dobot.GetQMatrix(goalDobot);

% ur3
goalUR3 = [-3.5 -0.1 0]; 
rock(1) = e.GetRock(goalUR3);
qMatrixUR3 = UR3.GetQMatrix(goalUR3);
% animate
for i = 1 : size(qMatrixUR3,1) % use ur3 due to its larger size 
    eStop(a)
    dobot.model.animate(qMatrixDobot(i,:));
    dobot.transformGripper(steps,true);
    eStop(a)
    UR3.model.animate(qMatrixUR3(i,:));
    UR3.transformGripper(steps,true);
end

% % dobot
goalDobot = e.getTileLocation(tileCounter);
qMatrixDobot = dobot.GetQMatrix(goalDobot);

% ur3
goalUR3 = e.hopperLocation;
qMatrixUR3 = UR3.GetQMatrix(goalUR3);
for i = 1 : size(qMatrixDobot,1) 
    eStop(a)
    dobot.model.animate(qMatrixDobot(i,:));
    dobot.transformGripper(steps,false);  
    eStop(a)
    ee = dobot.GeteeBase;
    e.UpdateLocation(tileCounter,ee,'tile');    
    eStop(a)
    UR3.model.animate(qMatrixUR3(i,:));
    UR3.transformGripper(steps,false);
    eStop(a)
    ee = UR3.GeteeBase;
    e.UpdateLocation(rock(1),ee,'rock');
end
e.UpdateLocation(tileCounter,transl([goalDobot(1) goalDobot(2) 0]),'tile');
e.UpdateLocation(rock(1),transl([e.hopperLocation(1) e.hopperLocation(2) 0]),'rock');
tileCounter = tileCounter + 1;

function eStop(a)
flag1 = 0; % flag2 = 0;
flag1 = readDigitalPin(a,'D2');
flag2 = readDigitalPin(a,'D3');
first = 0; second = 0;
while((flag1 == 1) || (flag2 == 1))
    towerLight(a,'red',1);
    first = readDigitalPin(a,'D3')
    while first == 1
        towerLight(a,'yellow',1);
        second = readDigitalPin(a,'D2')
        if second == 1 
            towerLight(a,'green',1);
            pause(5);
            flag1 = 0;
            flag2 = 0;            
            break
        end
        
    end
end

towerLight(a,'green',1);
end 

function towerLight(a,colour,lightSwitch) % 0 = off
switch colour
    case 'red'
        writeDigitalPin(a,'D8',0); 
        writeDigitalPin(a,'D9',0);
        writeDigitalPin(a,'D10',lightSwitch);        
    case 'yellow'
        writeDigitalPin(a,'D8',0);
        writeDigitalPin(a,'D9',lightSwitch); 
        writeDigitalPin(a,'D10',0);   
    case 'green'
        writeDigitalPin(a,'D8',lightSwitch); 
        writeDigitalPin(a,'D9',0);
        writeDigitalPin(a,'D10',0);         
end
end
