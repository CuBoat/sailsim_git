function [L_rudder,D_rudder]=rudderForce(v_boat,theta,omega,t,p)
%%%Calculates forces on rudder given the rudder is in the water

%velocity of rudder in water
v_rudder=v_boat+p.d_rudder*omega*[-sin(theta),cos(theta)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Set rudder angle
%p.angle_rRelb = setRudder(theta,p.desiredHeading);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%angle of attack of rudder in water
alpha_rudder=wrapTo2Pi(theta+p.angle_rRelb-atan2(v_rudder(2),v_rudder(1)));
[c_lift,c_drag]=C_LD(alpha_rudder,p);

%lift and drag on rudder
L_rudder=.5*p.rho_water*p.SA_rudder*norm(v_rudder)*...
    c_lift*[-v_rudder(2),v_rudder(1)];
D_rudder=.5*p.rho_water*p.SA_rudder*norm(v_rudder)*...
    c_drag*(-v_rudder);