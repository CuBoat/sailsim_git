%new way to try and simulate boat dynamics while also implemeting a 
%control system.

%Run ODE for small intervals and then make changes to rudder
%Allows data to be stored that it hard to take out of ODE

%first attempt at steering boat towards waypoint.
%will also try to calculate v_T (velocity of the boat towards the target)

% clc; clear;
% close all 

%length of simulation
tfin = 100;
tdiv = 0.01;
n=tfin/tdiv;
tspan = linspace(0,tfin,n);

%initial conditions
x0=0; y0=0; th0=45*(pi/180); xdot0=0; ydot0=0; thdot0=0; 
th0 = wrapTo2Pi(th0);
z0=[x0,y0,th0,xdot0,ydot0,thdot0]';
preverror = 0;

%parameters
p=setBoatParam;

%calculate polar plot
magVwind = norm(p.v_a);
polarPlot = polarDiagram(magVwind);
figure()
polar(polarPlot(1,:),polarPlot(2,:));
options=odeset('abstol',1e-4,'reltol',1e-4);

stateVar = zeros(n,6);
stateVar(1,:) = z0';
thetaDesired = zeros(n,1);
thetaDesired(1) = nan;
command = zeros(n,1);
command(1) = nan;
rudderPos = zeros(n,1);
rudderPos(1) = p.angle_rRelb;

f = @rhs;
thetaWind = atan2(p.v_a(2),p.v_a(1))+pi;
thetaWind = wrapTo2Pi(thetaWind);
%Target position for robot
p.T = [0,30]'; %[x;y], should probably be large compared to boat
closeEnough = 0.1;
jibe = 0;
RL = false;
LR = false;
thetaJibe = nan;
%tic
for i = 2: n
    
    poseBoat = stateVar(i-1,(1:3))';
    thetaBoat = wrapTo2Pi(stateVar(i-1,3));
    
    r = norm(poseBoat(1:2)-p.T);
    %Run optimization to find best heading
    if r > 1
        [thetaDesired(i),jibe,thetaJibe,RL,LR] = findBestHeading(poseBoat,p.T,thetaWind,polarPlot,jibe,thetaJibe,RL,LR);

        [p.angle_rRelb,preverror,command(i),jibe] = setRudder(thetaBoat,thetaDesired(i),preverror,jibe,RL,LR);
        rudderPos(i) = p.angle_rRelb;
        stateVar(i,:) = EulerIntegration(tdiv,stateVar(i-1,:),f,p);
        stateVar(i,3) = wrapTo2Pi(stateVar(i,3));
    else
       stateVar = stateVar(1:i-1,:);
       tspan = tspan(1:i-1);
       command = command(1:i-1);
       thetaDesired = thetaDesired(1:i-1);
       rudderPos = rudderPos(1:i-1);
       break;
    end
    
    %disp(n-i)
end
%toc
%stateVar(end,1)
%animate results
figure()
subplot(2,2,1)
plot(tspan,thetaDesired,'g',[0,tspan(end)],[thetaWind,thetaWind],'k');
xlim([0,tspan(end)]); ylim([0-.2, 2*pi+.2]);
title 'Desired Direction'
subplot(2,2,2)
plot(tspan,stateVar(:,3),'b')
title 'Boat Heading'
subplot(2,2,3)
plot(tspan,command,'r');
title 'command'
subplot(2,2,4)
plot(tspan,rudderPos,'c');
title 'Rudder Position'
speed = 2;
save = false;
animate(tspan,stateVar,thetaDesired,p,speed,save)



