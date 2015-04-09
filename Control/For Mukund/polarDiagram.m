function [polarPlot] = polarDiagram(v_airMag)
%%%Calculates and plots polar diagram
%   Inputs:
%       v_airMag   magnitude of wind velocity
%   Outputs:
%       writes file of 2XN with boat velocity angle on first row and
%       magnitude on second row

%set boat parameters
p=setBoatParam;
options=optimoptions('fsolve','Display','off','TolFun',1e-6,'TolX',1e-6);
%Resolution of plot
N=361;
%Angles of boat heading
Theta=linspace(0,2*pi,N);
%air velocity vector
p.v_a=[-v_airMag,0];
%initial guess of boat velocity
v_boat0=[0,1];
%Magnitudes of boat speed
v_boatMag=zeros(1,N);
theta_boat = zeros(1,N);
for k=1:N
    p.theta=Theta(k);
    %call nonlinear solve function
    f = @(v_boat)forces(v_boat,p);
    v_boat=fsolve(f,v_boat0,options);
    %set new guess to be previous velocity
    v_boat0=v_boat;
    theta_boat(k)=atan2(v_boat(2),v_boat(1));
    %ignore backward velocities
    if dot(v_boat,[cos(p.theta),sin(p.theta)]) > 0
        %calculate velocity magnitude
        v_boatMag(k)=norm(v_boat);
    else
        v_boatMag(k)=0;
    end
end

polarPlot = [Theta;v_boatMag];
% figure()
% polar(polarPlot(1),polarPlot(2));
%write to file
% dlmwrite('polarData',theta_boat,'delimiter','\t');
% dlmwrite('polarData',v_boatMag,'-append','delimiter','\t');
%plot
% figure
% hold on
% h=polar(Theta,v_boatMag,'r');
% title('Polar Diagram','fontsize',16)
% set(h,'linewidth',2);



function F=forces(v_boat,p)
%%%Velocity
%velocity of keel in water
v_keel=v_boat;
%sail velocity relative to air
v_sail=v_boat-p.v_a;

%%%Angle of attack
%angle of attack of keel in water
alpha_keel=wrapTo2Pi(p.theta-atan2(v_keel(2),v_keel(1)));
%determines if angle of attack of sail should be +5 or -5 deg.
if (wrapTo2Pi(atan2(v_sail(2),v_sail(1))-p.theta)) < pi
    alpha_sail=355*pi/180;
else
    alpha_sail=5*pi/180;
end

%%%Lift Drag Coefficient
[c_liftKeel,c_dragKeel]=C_LD(alpha_keel,p);
[c_liftSail,c_dragSail]=C_LD(alpha_sail,p);

%%%Induced drag
% c_dragiKeel=c_liftKeel^2/(pi*p.AR_keel*p.e_keel);
% c_dragKeel=c_dragKeel+c_dragiKeel;
% c_dragiSail=c_liftSail^2/(pi*p.AR_sail*p.e_sail);
% c_dragSail=c_dragSail+c_dragiSail;

%%%Lift and Drag forces
%lift and drag on keel
L_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_liftKeel*[-v_keel(2),v_keel(1)];
D_keel=.5*p.rho_water*p.SA_keel*norm(v_keel)*...
    c_dragKeel*(-v_keel);
%lift and drag on sail
L_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_liftSail*[-v_sail(2),v_sail(1)];
D_sail=.5*p.rho_air*p.SA_sail*norm(v_sail)*...
    c_dragSail*(-v_sail);
%Hull resistance
R_hull=2.48*norm(v_boat)^2*(-v_boat);

%%% Sum of forces
F=L_keel+D_keel+L_sail+D_sail+R_hull;