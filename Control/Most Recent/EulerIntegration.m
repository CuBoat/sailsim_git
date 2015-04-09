function [Z] = EulerIntegration(t,Z_prev,f,p)

%find rate of change for states, evaluates "plane_ODE"
Zdot = feval(f,t,Z_prev',p)';

%Z is equal to the previous Z plus the rate of change of Z times the time
%step
Z = Zdot * t + Z_prev;

end