

% clc; clear;
% close all 

%length of simulation
t = 0;
tfin =1;
tdiv = 0.01;
n=tfin/tdiv;
tspan = linspace(0,tfin,n);

%initial conditions
x0=0; y0=0; th0=0*(pi/180); xdot0=0; ydot0=0; thdot0=0; 
th0 = wrapTo2Pi(th0);
z0=[x0,y0,th0,xdot0,ydot0,thdot0]';
prevError = 0;

%parameters
p=setBoatParam;

%calculate polar plot
magVwind = norm(p.v_a);
polarPlot = polarDiagram(magVwind);
% figure()
% polar(polarPlot(1,:),polarPlot(2,:));

stateVar = zeros(n,6);
stateVar(1,:) = z0';
thetaDesired = zeros(n,1);
thetaDesired(1) = nan;
command = zeros(n,1);
command(1) = nan;
rudderPos = zeros(n,1);
rudderPos(1) = p.angle_rRelb;
error = zeros(n,1);
error(1) = nan;

f = @rhs;
thetaWind = atan2(p.v_a(2),p.v_a(1))+pi;
thetaWind = wrapTo2Pi(thetaWind);

%Target position for robot
% p.T = {[20,0];[20,20];[0,20];[0,0]}; %[x;y], should probably be large compared to boat
p.T = {[20,0]};
j = 1;
closeEnough = 2;
%tic
for i = 2: n
    
    poseBoat = stateVar(i-1,(1:3))';
    thetaBoat = wrapTo2Pi(stateVar(i-1,3));
    
    
    %Run optimization to find best heading
    if j <= length(p.T) 
        t = t + tdiv;
        
        curTarget = cell2mat(p.T(j))';
        r = norm(poseBoat(1:2)-curTarget);
        
        [thetaDesired(i)] = findBestHeading(poseBoat,curTarget,thetaWind,polarPlot);
        [error(i)] = calcError(thetaDesired(i),thetaBoat,thetaWind);
        [p.angle_rRelb,command(i),prevError] = setRudder(error(i),prevError);
        rudderPos(i) = p.angle_rRelb;
        stateVar(i,:) = EulerIntegration(tdiv,stateVar(i-1,:),f,p);
        stateVar(i,3) = wrapTo2Pi(stateVar(i,3));
        if r < closeEnough
            j = j +1;
        end
    else
        stateVar = stateVar(1:i-1,:);
        tspan = tspan(1:i-1);
        command = command(1:i-1);
        thetaDesired = thetaDesired(1:i-1);
        rudderPos = rudderPos(1:i-1);
        break;
    end
    
    disp(n-i)
end
% toc
stateVar(end,1)
% animate results
figure(1)
subplot(2,2,1)
plot(tspan,thetaDesired,'g',[0,tspan(end)],[thetaWind,thetaWind],'k');
xlim([0,tspan(end)]); ylim([0-.2, 2*pi+.2]);
title 'Desired Direction'
subplot(2,2,2)
plot(tspan,stateVar(:,3),'b')
title 'Boat Heading'
subplot(2,2,3)
plot(tspan,command,'r');
title 'Command'
subplot(2,2,4)
plot(tspan,rudderPos,'c');
title 'Rudder Position'

speed = 2;
recordOptions.save = false;
recordOptions.frameRate = 10;
recordOptions.filename = 'Box_Waypoint_4x';
recordOptions.profile = 'MPEG-4';

animate(tspan,stateVar,thetaDesired,p,speed,recordOptions)



