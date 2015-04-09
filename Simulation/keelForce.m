function [L_keel,D_keel]=keelForce(v_boat,theta,omega,t,p)
%%%Calculates lift drag forces on keel

%velocity of keel in water
v_keel=v_boat+p.d_keel*omega*[-sin(theta),cos(theta)];

%angle of attack of keel in water
alpha_keel=wrapTo2Pi(theta-atan2(v_keel(2),v_keel(1)));
[c_lift,c_drag]=C_LD(alpha_keel,p);

%lift and drag on keel
L_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_lift*[-v_keel(2),v_keel(1)];
D_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_drag*(-v_keel);
