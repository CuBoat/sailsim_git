function [rudderAng] = ctrl(poseBoat,theta_wind,T)
xT = T(1); yT = T(2);
xB = poseBoat(1); yB = poseBoat(2); thetaB = poseBoat(3);


theta_TB = atan2((yT-yB),(xT-xB));

error = sin(theta_TB - thetaB);
 
angle = 5;

if error > 0%deadBand
    rudderAng = -angle * (pi/180); %boat turn CCW

elseif error < 0%-deadBand 
    rudderAng = angle * (pi/180); %boat turn CW

else 
    rudderAng = 0;
end
    

end