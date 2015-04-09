%new way to try and simulate boat dynamics while also implemeting a 
%control system.

%Run ODE for small intervals and then make changes to rudder
%Allows data to be stored that it hard to take out of ODE

clc; clear;
close all 

%length of simulation
tfin = 20;

%say we can change rudder position once per second
m = 1;

%n is number of time control program will run
n = tfin/m;

tspan = linspace(0,tfin,n);

%initial conditions
x0=0; y0=0; th0=0*(pi/180); xdot0=0; ydot0=0; thdot0=0; 
z0=[x0,y0,th0,xdot0,ydot0,thdot0]';


%parameters
p=setBoatParam;
p.desiredHeading = 50*pi/180;

options=odeset('abstol',1e-4,'reltol',1e-4);

%resolution of animation
res = 25;

stateVar = zeros(n*res,6);
t_tot = zeros(n*res,1);



for i = 1: n
   
    tspan_ctrl = linspace(i-1,i,res);
    theta = z0(3);
    theta_wind = atan2(p.v_a(2),p.v_a(1));
    
    p.angle_rRelb = ctrl(theta,theta_wind);
    
    [t,zarray]=ode23(@rhs,tspan_ctrl,z0,options,p);
    
    t_tot((i-1)*res+1:i*res) = t;
    stateVar((i-1)*res+1:i*res,:) = zarray;
    
    z0 = zarray(res,:);
end

%animate results
animate(t_tot,stateVar,p)



