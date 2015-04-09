function p=setBoatParam
%%%initializes various boat parameters in p structure

%%%p.accuracy defines the accuracy with which lift and drag coeff. are
%%%interpolated
%1: most accurate but slower than method 3 (using pchip interpolation)
%2: semi accurate but slowest method by far (using chebychev interpolation)
%3: less accurate but fastest (approx. Clift Cdrag as sinusoidal)
p.accuracy=3;      

%sail
p.d_sail=0; %distance from C.O.M. to sail [m] (positive is infront COM)
p.SA_sail=.2; %surface area sail [m^2]
p.angle_sRelb=-20*pi/180; %angle of sail relative to boat [rad]

%keel
p.d_keel=0; %distance from C.O.M. to keel [m] (positive is infront of COM)
p.SA_keel=0.1; %surface area keel [m^2]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Control Stuff
p.desiredHeading = 0*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%rudder
p.d_rudder=-0.5; %distance from C.O.M. to rudder [m] (positive is behind COM)
p.SA_rudder=0.01; %surface area rudder [m^2]
p.angle_rRelb=0*pi/180; %angle of rudder relative to boat [rad]

p.v_a=[0,10]; %x-y velocity componenets of air [m/s]
p.mass=3; %mass of boat [kg]
p.I=(1/12)*p.mass; %moment of inertia of boat about COM [kg*m^2]
p.rho_air=1.2; %density of air [kg/m^3]
p.rho_water=1000; %density of water [kg/m^3]

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

p.paraDrag=.2; %parasitic drag
drag=drag+p.paraDrag; %adjusted drag

p.C0=1;  % nominal lift coefficient for sinusoidal lift/drag approx.

%%% Fit a pchip piecewise-polynomial curve fit to the data
% This is good because it preserves local maximum and minimum
% Another option would be to replace pchip() with spline(), which would
% produce a more smooth curve, but would add new peaks to the data.
p.lift = pchip(angle,lift);
p.drag = pchip(angle,drag);

%Chebychev curvefitting very slow and less accurate than pchip
chebOrder = 25;
ppEval.angle = linspace(0,360,500);
ppEval.lift = ppval(p.lift,ppEval.angle);
ppEval.drag = ppval(p.drag,ppEval.angle);

io.input = ppEval.angle;
io.output = ppEval.lift;
[p.chebLift, p.domain] = chebyshevFit(io, chebOrder);
io.output = ppEval.drag;
p.chebDrag = chebyshevFit(io, chebOrder);
