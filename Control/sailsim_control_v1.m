

function sailsim_control_v1()
%simulates sailboat as keel, hull, & rudder (all airfoils) in 2D

clc
close all

%1: more accurate but slower (using pchip interpolation of Clift Cdrag
%2: less accurate but faster (approx. Clift Cdrag as sinusoidal)
p.accuracy=1;                

%initial conditions
tspan=linspace(0,3,500); %timespan of simulation [s]
p.rudderAng = zeros(length(tspan),1);
x0=0; y0=0; th0=0; xdot0=0; ydot0=0; thdot0=0; %initial pose and velocities
z0=[x0,y0,th0,xdot0,ydot0,thdot0]';

%%% Tabulated Data for NACA 0015 airfoil:
angle = [0,10,15,17,23,33,45,55,70,80,90,100,110,120,130,...
    140,150,160,170,180,190,200,210,220,230,240,250,260,...
    270,280,290,305,315,327,337,343,345,350,360]';

lift = [0,0.863,1.031,0.58,.575,.83,.962,.8579,.56,.327,...
    .074,-.184,-.427,-.63,-.813,-.898,-.704,-.58,-.813,0,...
    .813,.58,.704,.898,.813,.63,.427,.184,-.074,-.327,...
    -.56,-.8579,-.962,-.83,-.575,-.58,-1.031,-.863,0]';

drag = [0,.021,.058,.216,.338,.697,1.083,1.421,1.659,1.801,...
    1.838,1.758,1.636,1.504,1.26,.943,.604,.323,.133,0,...
    .133,.323,.604,.943,1.26,1.504,1.636,1.758,1.838,1.801,...
    1.659,1.421,1.083,.697,.338,.216,.058,.021,0]';
p.paraDrag=.05; %parasitic drag
drag=drag+p.paraDrag;

p.lift = pchip(angle,lift);
p.drag = pchip(angle,drag);

%%plotting lift/drag coeff
% ppEval.angle = linspace(0,360,1000);
% ppEval.lift = ppval(p.lift,ppEval.angle);
% ppEval.drag = ppval(p.drag,ppEval.angle);
% plot(ppEval.angle,ppEval.drag)

%parameters
p.v_a=[10,0]; %x-y velocities of air [m/s]
p.angle_sRelb=20*pi/180; %angle of sail relative to boat [rad]
p.angle_rRelb=0*pi/180; %angle of rudder relative to boat [rad]
p.mass=3; %mass of boat [kg]
p.d_rudder=.5; %distance from C.O.M. to rudder [m] (positive is behind COM)
p.d_sail=0; %distance from C.O.M. to sail [m] (positive is infront COM)
p.d_keel=0; %distance from C.O.M. to keel [m] (positive is infront of COM)
p.I=(1/12)*p.mass*2*p.d_rudder^2; %moment of inertia of boat about COM [kg*m^2]
p.rho_air=1.2; %density of air [kg/m^3]
p.rho_water=1000; %density of water [kg/m^3]
p.SA_sail=.2; %surface area sail [m^2]
p.SA_keel=0.1; %surface area keel [m^2]
p.SA_rudder=0.01; %surface area rudder [m^2]

%added desired direction
p.desiredHeading = 0;
p.i = 1; %keep track of ODE solver loop iterations

options=odeset('abstol',1e-4,'reltol',1e-4);
%predict future
[t,zarray]=ode23(@rhs,tspan,z0,options,p);
%animate results
animate(t,zarray,p)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function zdot=rhs(t,z,p)
th=z(3); %angle of boat heading relative to x-axis
v_b=z(4:5)'; %2D vector of boat
thd=z(6); %angular velocity of boat
v_rel=p.v_a-(v_b+p.d_sail*thd*[-sin(th),cos(th)]); %air velocity as percieved by sail
v_rudder=v_b+p.d_rudder*thd*[sin(th),-cos(th)]; %velocity of rudder in water
v_keel=v_b+p.d_keel*thd*[-sin(th),cos(th)]; %velocity of keel in water

%angle of attack of sail in air
alpha_sail=wrapTo2Pi(th+p.angle_sRelb-atan2(-v_rel(2),-v_rel(1)));
[c_lift,c_drag]=LiftDrag(alpha_sail,p);
%lift and drag on sail
L_sail=.5*p.rho_air*p.SA_sail*norm(v_rel)*...
    c_lift*[v_rel(2),-v_rel(1)];
D_sail=.5*p.rho_air*p.SA_sail*norm(v_rel)*...
    c_drag*v_rel;

%angle of attack of keel in water
alpha_keel=wrapTo2Pi(th-atan2(v_keel(2),v_keel(1)));
[c_lift,c_drag]=LiftDrag(alpha_keel,p);
%lift and drag on keel
L_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_lift*[-v_keel(2),v_keel(1)];
D_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_drag*(-v_keel);

%Set Rudder angle here
p.angle_rRelb = setRudder(th,p.desiredHeading);
p.rudderAng(p.i) = p.angle_rRelb;
p.i = p.i + 1;
%%%%%%%%%%%%%%%%%%%%%

%angle of attack of rudder in water
alpha_rudder=wrapTo2Pi(th+p.angle_rRelb-atan2(v_rudder(2),v_rudder(1)));
[c_lift,c_drag]=LiftDrag(alpha_rudder,p);
%lift and drag on rudder
L_rudder=.5*p.rho_water*p.SA_rudder*norm(v_rudder)*...
    c_lift*[-v_rudder(2),v_rudder(1)];
D_rudder=.5*p.rho_water*p.SA_rudder*norm(v_rudder)*...
    c_drag*(-v_rudder);

%LMB
vdot=(L_sail+D_sail+L_keel+D_keel+L_rudder+D_rudder)/p.mass;

%AMB
wdot=(-p.d_rudder*cos(th)*(L_rudder(2)+D_rudder(2))+...
    p.d_rudder*sin(th)*(L_rudder(1)+D_rudder(1))+...
    p.d_keel*cos(th)*(L_keel(2)+D_keel(2))-...
    p.d_keel*sin(th)*(L_keel(1)+D_keel(1))+...
    p.d_sail*cos(th)*(L_sail(2)+D_sail(2))-...
    p.d_sail*sin(th)*(L_sail(1)+D_sail(1)))/p.I;

zdot=[z(4:6);vdot';wdot];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function animate(t,zarray,p)
%create plots and animation

%figure
%plot(t,sqrt(zarray(:,4).^2+zarray(:,5).^2));
xlabel('time','fontsize',16);
ylabel('boat velocity magnitude','fontsize',16);

%figure
%plot(zarray(:,1),zarray(:,2));
xlabel('x-position boat','fontsize',16);
ylabel('y-position boat','fontsize',16);

%animation
figure('units','normalized','outerposition',[0 0 1 1],'color',[.7,.9,1])
annotation('textarrow',[.1,.1+.1*p.v_a(1)/norm(p.v_a)],...
    [.9,.9+.1*p.v_a(2)/norm(p.v_a)],'string','Velocity_a_i_r',...
    'fontsize',18,'linewidth',3);
for m=1:length(t)
    cla
    th=zarray(m,3);
    x=zarray(m,1);
    y=zarray(m,2);
    d=p.d_rudder;
    %plot > rotate > translate hull
    xh=[-d,0,d,0,-d,-d];
    yh=[-.1*d,-.2*d,0,.2*d,.1*d,-.1*d];
    xhp=xh*cos(th)-yh*sin(th)+x;
    yhp=xh*sin(th)+yh*cos(th)+y;
    fill(xhp,yhp,'k');
    %plot > rotate > translate keel
    xk=[-.5*d,.5*d];
    yk=[0,0];
    xkp=xk*cos(th)-yk*sin(th)+x;
    ykp=xk*sin(th)+yk*cos(th)+y;
    plot(xkp,ykp,':b','linewidth',5);
    %plot > rotate > translate sail
    xs=[-.5*d,.5*d];
    ys=[0,0];
    xsp=xs*cos(th+p.angle_sRelb)-ys*sin(th+p.angle_sRelb)+x;
    ysp=xs*sin(th+p.angle_sRelb)+ys*cos(th+p.angle_sRelb)+y;
    plot(xsp,ysp,'w','linewidth',5);
    %plot > rotate > translate rudder
    xr=[-.2*d,.2*d];
    yr=[0,0];
%     xrp=xr*cos(th+p.angle_rRelb)-...
%         yr*sin(th+p.angle_rRelb)+x-d*cos(th);
%     yrp=xr*sin(th+p.angle_rRelb)+...
%         yr*cos(th+p.angle_rRelb)+y-d*sin(th);
    
    xrp=xr*cos(th+rudderAng(m))-...
        yr*sin(th+rudderAng(m))+x-d*cos(th);
    yrp=xr*sin(th+rudderAng(m))+...
        yr*cos(th+rudderAng(m))+y-d*sin(th);
    plot(xrp,yrp,'w','linewidth',5);
    
    %manage axes
    hold on
    axis equal
    axis off
    minx=min(zarray(:,1))-d; miny=min(zarray(:,2))-d;
    maxx=max(zarray(:,1))+d; maxy=max(zarray(:,2))+d;
    axis([minx,maxx,miny,maxy]);
    pause(0.01)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c_lift,c_drag]=LiftDrag(alpha,p)
%calculates coeff. of lift and drag at a certain alpha
if p.accuracy==1
    alpha=alpha*180/pi;
    c_lift=ppval(p.lift,alpha);
    c_drag=ppval(p.drag,alpha);
else
    c_lift=sin(2*alpha);
    c_drag=p.paraDrag+(1-cos(2*alpha));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function angle=wrapTo2Pi(angle)
%convert angle to be in range [0,2pi]
angle=rem(angle,2*pi);
if angle<0
    angle=2*pi+angle;
end
