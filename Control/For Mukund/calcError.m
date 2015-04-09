function [error] = calcError(thetaDesired,thetaBoat,thetaWind)

thetaDesired = thetaDesired * (180/pi);
thetaBoat = thetaBoat * (180/pi);
thetaWind = thetaWind * (180/pi);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

error = thetaDesired - thetaBoat;
    if error > 180 || error < -180
        if error > 0
            error = error - 360;
        elseif error < 0
            error = error + 360;
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

thetaDtoW = thetaDesired - thetaWind;
    if thetaDtoW > 180 || thetaDtoW < -180
        if thetaDtoW > 0
            thetaDtoW = thetaDtoW - 360;
        elseif thetaDtoW < 0
            thetaDtoW = thetaDtoW + 360;
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if  error > 0 && thetaDtoW < error && thetaDtoW > 0 
        error = error - 360;

    elseif  error < 0 && thetaDtoW > error && thetaDtoW < 0
        error = error + 360;
    end
    
error = error * (pi/180);    
end