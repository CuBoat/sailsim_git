function [thetaBest] = findBestHeading(poseBoat,T,thetaWind,polarPlot)

%Initialization
r = T - poseBoat(1:2); %distance vector from the boat to the target

thetaBoat = poseBoat(3);    %Boat heading

Pc = 5;                 %Beating parameter, changes width of jibing pattern

n = 1 + Pc/norm(r); %used determine if its time to jibe


%Optimization to find the best boat heading in order to have largest
%Velocity towards the target

%Do right hand side of polar plot first
alpha = 360;  %angle of polar plot being checked, start at zero
delalpha = 1; %step size of optimization through polar plot, in degrees

v_T_R_max = 0; %Set max velcity towards the target zero to start

thetaBoat_R_max = thetaWind + alpha;  %set best boat angle to angle of wind
                                      %essentially the same as setting it
                                      %to zero because the best angle
                                      %should never be directly into the
                                      %wind
%Run optimization for 180 <= alpha <= 360, aka the right half of the polar
%plot

    while alpha >= 180
        %function to calculate velocity of boat given wind speed and angle
        %relative to the wind
        vB = calcvb(alpha,polarPlot,thetaWind);
        
        %Calculate velocity towards the target
        v_T_R = vB' * (r/norm(r));
        
        
        if v_T_R > v_T_R_max
           %if the vT is the largest so far, set it as new highest
           v_T_R_max = v_T_R;
           %set thetaBoat to angle corresponding to max vT
           thetaBoat_R_max = thetaWind + alpha*(pi/180); 
        end
        
        %Incease alpha to continue through polar plot
        alpha = alpha - delalpha;
    end

thetaBoat_R_max = wrapTo2Pi(thetaBoat_R_max);
%Now run optimization for LHS of polar plot

alpha = 0;

v_T_L_max = 0;

thetaBoat_L_max = thetaWind + alpha;

    while alpha <= 180
        %function to calculate velocity of boat given wind speed and angle
        %relative to the wind
        vB = calcvb(alpha,polarPlot,thetaWind);
        v_T_L = vB' * (r/norm(r));
        
        if v_T_L > v_T_L_max
           %if the vT is the largest so far, set it as new highest
           v_T_L_max = v_T_L;
           %set thetaBoat to angle corresponding to max vT
           thetaBoat_L_max = thetaWind + alpha*(pi/180); 
        end
        
        alpha = alpha + delalpha;
    end

thetaBoat_L_max = wrapTo2Pi(thetaBoat_L_max);

%Chosing which angle to use

    if abs(thetaBoat_R_max - thetaBoat) < abs(thetaBoat_L_max - thetaBoat)
        if (v_T_R_max * n) < v_T_L_max
           thetaBest = thetaBoat_L_max;
           %disp('right ---> left')
        else
            thetaBest = thetaBoat_R_max;
            %disp('right')  
        end
    else
        if (v_T_L_max * n) < v_T_R_max
            thetaBest = thetaBoat_R_max;
            %disp('left ---> right')
        else
            thetaBest = thetaBoat_L_max; 
            %disp('left'); 
        end
    end
    %thetaBest
    %thetaBest = wrapTo2Pi(thetaBest);
end