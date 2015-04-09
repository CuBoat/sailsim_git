function animate(t,zarray,thetaDesired,p,speed,recordOptions)
%%% Animates boat's motion

% figure
% plot(t,sqrt(zarray(:,4).^2+zarray(:,5).^2));
% xlabel('time','fontsize',16);
% ylabel('boat velocity magnitude','fontsize',16);
% 
% figure
% plot(t,zarray(:,6));
% xlabel('time','fontsize',16);
% ylabel('Angular velocity','fontsize',16);
% 
% figure
% plot(t,zarray(:,3));
% xlabel('time','fontsize',16);
% ylabel('Angle','fontsize',16);
% 
% 
% figure
% plot(zarray(:,1),zarray(:,2));
% xlabel('x-position boat','fontsize',16);
% ylabel('y-position boat','fontsize',16);

%animation
H=figure('units','normalized','outerposition',[0 0 1 1],'color',[.7,.9,1]);
%wind vector
annotation('textarrow',[.1,.1+.1*p.v_a(1)/norm(p.v_a)],...
    [.9,.9+.1*p.v_a(2)/norm(p.v_a)],'string','Velocity_a_i_r',...
    'fontsize',18,'linewidth',3);

%Animation
d=max(abs([p.d_sail,p.d_keel,p.d_rudder]));
ds=p.d_sail; dk=p.d_keel; dr=p.d_rudder;

hull = fill(nan,nan,'k');
A = get(H,'Children');
hold on
traject = plot(nan,nan,'k','linewidth',2);
keel = plot(nan,nan,':b','linewidth',5);
sail = plot(nan,nan,'w','linewidth',5);
rudder = plot(nan,nan,'w','linewidth',5);
target = plot(nan,nan,'*r','markersize',25);
thetaDesiredrep = plot(nan,nan,'g','linewidth',3);
thetaDesiredline = [0,1];

wayPoints = cell2mat(p.T);
minx=min([zarray(:,1)',wayPoints(:,1)'])-d; miny=min([zarray(:,2)',wayPoints(:,2)'])-d;
maxx=max([zarray(:,1)',wayPoints(:,1)'])+d; maxy=max([zarray(:,2)',wayPoints(:,2)'])+d;

a = ((1/recordOptions.frameRate)/t(2));
last = 1;
u = 0;

    if recordOptions.save
        writerObj = VideoWriter(recordOptions.filename,recordOptions.profile);
        writerObj.FrameRate = recordOptions.frameRate * speed;
        open(writerObj);
    end
    
tic
for m=1:length(t)
    %cla
    th=zarray(m,3);
    x=zarray(m,1);
    y=zarray(m,2);
    %plot trajectory so far
    %plot(zarray(1:m,1),zarray(1:m,2),'k','linewidth',2);
    set(traject,'XData',zarray(1:m,1),'YData',zarray(1:m,2));
    set(target,'XData',wayPoints(:,1),'YData',wayPoints(:,2));
    xthD = thetaDesiredline * cos(thetaDesired(m)) + x;
    ythD = thetaDesiredline * sin(thetaDesired(m)) + y;
    set(thetaDesiredrep,'XData',xthD,'YData',ythD);
    
    
    %plot Target 
    %plot(p.T(1),p.T(2),'*','markersize',10);
    
    %plot > rotate > translate hull
    xh=[-d,0,d,0,-d,-d];
    yh=[-.1*d,-.2*d,0,.2*d,.1*d,-.1*d];
    xhp=xh*cos(th)-yh*sin(th)+x;
    yhp=xh*sin(th)+yh*cos(th)+y;
    %fill(xhp,yhp,'k');
    set(hull,'XData',xhp,'YData',yhp)
    
    %plot > rotate > translate keel
    xk=[-.5*d,.5*d]+dk;
    yk=[0,0];
    xkp=xk*cos(th)-yk*sin(th)+x;
    ykp=xk*sin(th)+yk*cos(th)+y;
    %plot(xkp,ykp,':b','linewidth',5);
    set(keel,'XData',xkp,'YData',ykp);
    
    %plot > rotate > translate sail
    xs=[-.5*d,.5*d]+ds;
    ys=[0,0];
    xsp=xs*cos(th+p.angle_sRelb)-ys*sin(th+p.angle_sRelb)+x;
    ysp=xs*sin(th+p.angle_sRelb)+ys*cos(th+p.angle_sRelb)+y;
    %plot(xsp,ysp,'w','linewidth',5);
    set(sail,'XData',xsp,'YData',ysp);
    
    %plot > rotate > translate rudder
    xr=[-.2*d,.2*d];
    yr=[0,0];
    xrp=xr*cos(th+p.angle_rRelb)-yr*sin(th+p.angle_rRelb)+x+dr*cos(th);
    yrp=xr*sin(th+p.angle_rRelb)+yr*cos(th+p.angle_rRelb)+y+dr*sin(th);
    %plot(xrp,yrp,'w','linewidth',5);
    set(rudder,'XData',xrp,'YData',yrp);
   
    set(A,'Visible','off','DataAspectRatio',[1 1 1],...
           'DataAspectRatioMode','manual','PlotBoxAspectRatio',...
           [3 4 4],'PlotBoxAspectRatioMode','manual',...
           'XLIM',[minx,maxx],'YLIM',[miny,maxy]); 
       
    if recordOptions.save && (m-last) >= a || recordOptions.save && m == 1 %|| recordOptions.save && m == length(t)
        u = u + 1;
        last = m;
        frame = getframe(H);   
        writeVideo(writerObj,frame);
    end
    
    pTime = (t(2)*m/speed-toc);
    pause(pTime)
    %pause(0.0001)
end
    if recordOptions.save
        writerObj.FrameCount
        writerObj.Duration
        
        close(writerObj);
    end
toc
disp(t(m));