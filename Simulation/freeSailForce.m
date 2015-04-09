function [L_sail,D_sail]=freeSailForce(v_boat,theta,omega,t,p)
%%%calculates sail force assuming the sail always takes a 5 deg angle of
%%%attack to the relative wind such that the lift points towards the bow

%sail velocity relative to air
v_sail=v_boat+p.d_sail*omega*[-sin(theta),cos(theta)]-p.v_a;

%determines if angle of attack should be +5 or -5 deg.
if (wrapTo2Pi(atan2(v_sail(2),v_sail(1))-theta)) < pi
    alpha_sail=355*pi/180;
else
    alpha_sail=5*pi/180;
end

[c_lift,c_drag]=C_LD(alpha_sail,p);

%lift and drag on sail
L_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_lift*[-v_sail(2),v_sail(1)];
D_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_drag*(-v_sail);
