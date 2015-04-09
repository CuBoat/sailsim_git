function [vB] = calcvb(alpha,polarPlot,thetaWind)

%[~,N] = size(polarPlot);
%thetas = polarPlot(1,:);
velocities = polarPlot(2,:)

vBmag = velocities(alpha+1);

alpha = alpha * (pi/180);



% for i = 1 : N
%    if alpha <= thetas(i)
%       if alpha == thetas(i)
%           vBmag = velocities(i);
%       elseif alpha < thetas(i)
%           vBmag = interp1([thetas(i),thetas(i-1)],[velocities(i),velocities(i-1)],alpha);
%           
%       end
%       break;
%    end
% end

vB = [vBmag*cos(thetaWind+alpha),vBmag*sin(thetaWind+alpha)]';

end