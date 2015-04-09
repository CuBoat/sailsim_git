function angle=wrapTo2Pi(angle)
%%%convert angle to be in range [0,2pi]

angle=rem(angle,2*pi);
if angle<0
    angle=2*pi+angle;
% if angle > 2*pi
%     angle  = angle - 2*pi;
% end
end