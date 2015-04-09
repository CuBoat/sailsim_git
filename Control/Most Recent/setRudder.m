
function [rudderAng,command,prevError] = setRudder(error,prevError)

rudderMax = 15 * (pi/180);
kp=1;
kd=0;
%ki=1;

errorP = kp * error;
errorD  = kd * (error - prevError);
prevError = error;
%errorI = errorI + error*ki;

command = errorP + errorD;% + errorI;


    if command > 0 
        rudderAng = -command;
    elseif command < 0
        rudderAng = -command;
    else
        rudderAng = 0;
    end


    if abs(rudderAng) > rudderMax
        if rudderAng < 0
            rudderAng = -rudderMax;
        elseif rudderAng > 0
            rudderAng = rudderMax;
        end
    end
  
end