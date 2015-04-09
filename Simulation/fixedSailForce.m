function [L_sail,D_sail]=fixedSailForce(v_boat,theta,omega,t,p)
%%%Calculates lift drag forces on a sail at a fixed angle relative to the
%%%boat

%sail velocity relative to air
v_sail=v_boat+p.d_sail*omega*[-sin(theta),cos(theta)]-p.v_a;

%angle of attack of sail in air
alpha_sail=wrapTo2Pi(th+p.angle_sRelb-atan2(v_sail(2),v_sail(1)));
[c_lift,c_drag]=C_LD(alpha_sail,p);

%lift and drag on sail
L_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_lift*[-v_sail(2),v_sail(1)];
D_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_drag*(-v_sail);
