function main()
%%%simulates sailboat as keel, hull, & rudder (all airfoils) in 2D

clc
close all        

%initial conditions
tspan=linspace(0,20,200); %timespan of simulation [s]
x0=0; y0=0; th0=120*(pi/180); xdot0=0; ydot0=0; thdot0=0; %initial pose and (d/dt)pose
z0=[x0,y0,th0,xdot0,ydot0,thdot0]';

%parameters
p=setBoatParam;

options=odeset('abstol',1e-4,'reltol',1e-4);

%predict future
[t,zarray]=ode23(@rhs,tspan,z0,options,p);

%animate results
animate(t,zarray,p)