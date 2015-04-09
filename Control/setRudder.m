
function [rudderAng,preverror,command,jibe,RL,LR] = setRudder(actualHeading,desiredHeading,preverror,jibe,RL,LR)

rudderMax = 15 * (pi/180);

kp=1;
kd=1;

% if actualHeading > pi
%     actualHeading = actualHeading - 2*pi;
% end
% if desiredHeading > pi
%     desiredHeading = desiredHeading - 2*pi;
% end
%error = sin(desiredHeading - actualHeading);

if ~jibe
    error = desiredHeading - actualHeading;
     
    if abs(error) > pi
        if error > 0
            error = error - 2*pi;
            if jibe
                error = -error;
            end
        elseif error < 0
            error = error + 2*pi;
            if jibe
                error = -error;
            end
        end
    end

command = error; %* kp + kd*(error-preverror);
preverror = error;

%command positive ---> rudder angle negative

    if command > 0%deadBand
        rudderAng = -command; %boat turn CCW

    elseif command < 0%-deadBand 
        rudderAng = -command; %boat turn CW

    else 
        rudderAng = 0;
    end

elseif jibe
    if RL
        rudderAng = rudderMax;
    elseif LR
        rudderAng = -rudderMax;
    end

    if abs(desiredHeading - actualHeading) < 5*(pi/180)
        jibe = 0;
        RL = false;
        LR = false;
    end
    command = nan;
end
    
    
if abs(rudderAng) > rudderMax
    if rudderAng < 0
        rudderAng = -rudderMax;
    elseif rudderAng > 0
        rudderAng = rudderMax;
    end
end
  
end