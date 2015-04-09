function zdot=rhs(t,z,p)
%%%Heart of the simulation: calculates rate of change at every timestep
%%%based on AMB and LMB
%disp(t)
theta=wrapTo2Pi(z(3)); %angle of boat heading relative to x-axis
v_boat=z(4:5)'; %2D vector of boat velocity
omega=z(6); %angular velocity of boat

[L_sail,D_sail]=freeSailForce(v_boat,theta,omega,t,p);
% [L_sail,D_sail]=fixedSailForce(v_boat,theta,omega,t,p);

[L_keel,D_keel]=keelForce(v_boat,theta,omega,t,p);

[L_rudder,D_rudder]=rudderForce(v_boat,theta,omega,t,p);
% [L_rudder,D_rudder]=airRudderForce(v_boat,theta,omega,t,p)

%sum of forces
Ftot=L_sail+D_sail+L_keel+D_keel+L_rudder+D_rudder;
%sum of moments
Mtot=p.d_rudder*cos(theta)*(L_rudder(2)+D_rudder(2))-...
    p.d_rudder*sin(theta)*(L_rudder(1)+D_rudder(1))+...
    p.d_keel*cos(theta)*(L_keel(2)+D_keel(2))-...
    p.d_keel*sin(theta)*(L_keel(1)+D_keel(1))+...
    p.d_sail*cos(theta)*(L_sail(2)+D_sail(2))-...
    p.d_sail*sin(theta)*(L_sail(1)+D_sail(1));

%LMB
vdot=Ftot/p.mass;

%AMB
wdot=Mtot/p.I;

%rate of change
zdot=[z(4:6);vdot';wdot];
