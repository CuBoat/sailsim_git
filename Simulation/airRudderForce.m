function [L_rudder,D_rudder]=airRudderForce(v_boat,theta,omega,t,p)
%%%Calculates the forces on the rudder given it is in the air

%velocity of rudder relative to air
v_rudder=v_boat+p.d_rudder*omega*[-sin(theta),cos(theta)]-p.v_a;

%angle of attack of rudder in air
alpha_rudder=wrapTo2Pi(theta+p.angle_rRelb-atan2(v_rudder(2),v_rudder(1)));
[c_lift,c_drag]=C_LD(alpha_rudder,p);

%lift and drag on rudder
L_rudder=.5*p.rho_air*p.SA_rudder*norm(v_rudder)*...
    c_lift*[-v_rudder(2),v_rudder(1)];
D_rudder=.5*p.rho_air*p.SA_rudder*norm(v_rudder)*...
    c_drag*(-v_rudder);