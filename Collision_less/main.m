
clear all;
close all;
clc
parameter;



time = [];
result = [];

tspan = [0 1.6];
options = odeset('Events',@collision,'RelTol',1e-12,'AbsTol',1e-12*ones(1,10),'Refine',15);
% options = odeset('Events',@collision,'RelTol',1e-5);
%%
global tttt;
global uuuu;
global iiii;
iiii=1;
for hosuu = 1:100
    if tspan(1)>=tspan(2)
        disp('シミュレーション終了!');
        break;
    elseif tspan(1)~=0
        disp(sprintf('impacttime=%f',time(end)));
    end
    
    [T,Q] = ode45(@rimless,tspan,q0,options);
    nt = length(T);
    tspan(1) = T(end);
    time=[time;T];
    result=[result;Q];
    q0 = change(Q(nt,:));   
    


end

%%
figure(1)
plot(time,result(:,3),'g',time,result(:,4),'m');
xlabel('Time [s]');
ylabel('Angular position [rad]');
legend('Angular position1','Angular position2');
grid on;

figure(2)
plot(time,result(:,7),'g',time,result(:,8),'m');
xlabel('Time [s]');
ylabel('Angular velocity [rad]');
legend('Angular velocity1','Angular velocity2');
grid on;

figure(3)
plot(result(:,3),result(:,7));
xlabel('th1 [rad]');
ylabel('dth1 [rad/s]');
grid on;
% % 
figure(5)
plot(tttt,uuuu);
axis([-inf,+inf,min(uuuu)-1,max(uuuu)+1]);
xlabel('time [s]');
ylabel('u');
grid on;



ii = 1;
for i = 1:length(time)
    if time(i) >= 0.001*(ii-1)
        Time(ii) = time(i);
        Result(ii,:) = result(i,:);
        ii = ii + 1;
    end
end
   
x=zeros(length(Time),18);
y=zeros(length(Time),18);
for i=1:length(Time)
    x0(i) = Result(i,1)+L*sin(Result(i,3));
    x(i,1) = Result(i,1);
    x(i,2) = x0(i) + L*sin(alpha1-Result(i,3));
    x(i,3) = x0(i) + L*sin(2*alpha1-Result(i,3));
    x(i,4) = x0(i) + L*sin(3*alpha1-Result(i,3));
    x(i,5) = x0(i) + L*sin(4*alpha1-Result(i,3));
    x(i,6) = x0(i) - L*sin(3*alpha1+Result(i,3));
    x(i,7) = x0(i) - L*sin(2*alpha1+Result(i,3));
    x(i,8) = x0(i) - L*sin(alpha1+Result(i,3));
    
    x(i,9) = x0(i)+0.85*L*sin(Result(i,4)+0*2*pi/10);
    x(i,10) = x0(i)+0.85*L*sin(Result(i,4)+1*2*pi/10);
    x(i,11) = x0(i)+0.85*L*sin(Result(i,4)+2*2*pi/10);
    x(i,12) = x0(i)+0.85*L*sin(Result(i,4)+3*2*pi/10);
    x(i,13) = x0(i)+0.85*L*sin(Result(i,4)+4*2*pi/10);
    x(i,14) = x0(i)+0.85*L*sin(Result(i,4)+5*2*pi/10);
    x(i,15) = x0(i)+0.85*L*sin(Result(i,4)+6*2*pi/10);
    x(i,16) = x0(i)+0.85*L*sin(Result(i,4)+7*2*pi/10);
    x(i,17) = x0(i)+0.85*L*sin(Result(i,4)+8*2*pi/10);
    x(i,18) = x0(i)+0.85*L*sin(Result(i,4)+9*2*pi/10);
    
    y0(i) = Result(i,2)+L*cos(Result(i,3));
    y(i,1) = Result(i,2);
    y(i,2) = y0(i) - L*cos(alpha1-Result(i,3));
    y(i,3) = y0(i) - L*cos(2*alpha1-Result(i,3));
    y(i,4) = y0(i) - L*cos(3*alpha1-Result(i,3));
    y(i,5) = y0(i) - L*cos(4*alpha1-Result(i,3));
    y(i,6) = y0(i) - L*cos(3*alpha1+Result(i,3));
    y(i,7) = y0(i) - L*cos(2*alpha1+Result(i,3));
    y(i,8) = y0(i) - L*cos(alpha1+Result(i,3));
    
    y(i,9) =  y0(i) - 0.85*L*cos(Result(i,4)+0*2*pi/10);
    y(i,10) = y0(i) - 0.85*L*cos(Result(i,4)+1*2*pi/10);
    y(i,11) = y0(i) - 0.85*L*cos(Result(i,4)+2*2*pi/10);
    y(i,12) = y0(i) - 0.85*L*cos(Result(i,4)+3*2*pi/10);
    y(i,13) = y0(i) - 0.85*L*cos(Result(i,4)+4*2*pi/10);
    y(i,14) = y0(i) - 0.85*L*cos(Result(i,4)+5*2*pi/10);
    y(i,15) = y0(i) - 0.85*L*cos(Result(i,4)+6*2*pi/10);
    y(i,16) = y0(i) - 0.85*L*cos(Result(i,4)+7*2*pi/10);
    y(i,17) = y0(i) - 0.85*L*cos(Result(i,4)+8*2*pi/10);
    y(i,18) = y0(i) - 0.85*L*cos(Result(i,4)+9*2*pi/10);
end
figure(4)

    plot0 = plot([-(x0(1)+5)*cos(phi) (x0(end)+5)*cos(phi)],[(x0(1)+5)*sin(phi) -(x0(end)+5)*sin(phi)],'k','LineWidth',2);hold on; %Ground

    plot1 = plot([x(1,1) x0(1)],[y(1,1) y0(1)],'r','LineWidth',4);hold on;
    plot2 = plot([x(1,2) x(1,6)],[y(1,2) y(1,6)],'b','LineWidth',4);hold on;
    plot3 = plot([x(1,3) x(1,7)],[y(1,3) y(1,7)],'b','LineWidth',4);hold on;
    plot4 = plot([x(1,4) x(1,8)],[y(1,4) y(1,8)],'b','LineWidth',4);hold on;
    plot5 = plot([x(1,5) x0(1)],[y(1,5) y0(1)],'b','LineWidth',4);hold on;
    
    inner1 = plot([x(1,9) x(1,14)],[y(1,9) y(1,14)],'m','LineWidth',3);hold on; % Line of inner wheel
    inner2 = plot([x(1,10) x(1,15)],[y(1,10) y(1,15)],'m','LineWidth',3);hold on;
    inner3 = plot([x(1,11) x(1,16)],[y(1,11) y(1,16)],'m','LineWidth',3);hold on;
    inner4 = plot([x(1,12) x(1,17)],[y(1,12) y(1,17)],'m','LineWidth',3);hold on;
    inner5 = plot([x(1,13) x(1,18)],[y(1,13) y(1,18)],'m','LineWidth',3);hold on;
    inner_cir = plot(0.85*cos(0:pi/20:2*pi)+x0(1), 0.85*sin(0:pi/20:2*pi)+y0(1),'m','LineWidth',3);hold on;
    
    rim1 = plot([x(1,1) x(1,2)],[y(1,1) y(1,2)],'b','LineWidth',4);hold on;
    rim2 = plot([x(1,2) x(1,3)],[y(1,2) y(1,3)],'b','LineWidth',4);hold on;
    rim3 = plot([x(1,3) x(1,4)],[y(1,3) y(1,4)],'b','LineWidth',4);hold on;
    rim4 = plot([x(1,4) x(1,5)],[y(1,4) y(1,5)],'b','LineWidth',4);hold on;
    rim5 = plot([x(1,5) x(1,6)],[y(1,5) y(1,6)],'b','LineWidth',4);hold on;
    rim6 = plot([x(1,6) x(1,7)],[y(1,6) y(1,7)],'b','LineWidth',4);hold on;
    rim7 = plot([x(1,7) x(1,8)],[y(1,7) y(1,8)],'b','LineWidth',4);hold on;
    rim8 = plot([x(1,8) x(1,1)],[y(1,8) y(1,1)],'b','LineWidth',4);hold on;
    


    axis equal;
    ax = gca;
    ax.SortMethod = 'depth';
    % ii = 1;
for i=1:10:length(Time)
    axis([x0(i)-2.0,x0(i)+2.0,-x0(i)*tan(phi)-0.6,-x0(i)*tan(phi)+2.4]);
    set(gcf,'Color','w');
    set(plot1,'Xdata',[x(i,1) x0(i)],'Ydata',[y(i,1) y0(i)]);
    set(plot2,'Xdata',[x(i,2) x(i,6)],'Ydata',[y(i,2) y(i,6)]);
    set(plot3,'Xdata',[x(i,3) x(i,7)],'Ydata',[y(i,3) y(i,7)]);
    set(plot4,'Xdata',[x(i,4) x(i,8)],'Ydata',[y(i,4) y(i,8)]);
    set(plot5,'Xdata',[x(i,5) x0(i)],'Ydata',[y(i,5) y0(i)]);
    
    set(rim1,'Xdata',[x(i,1) x(i,2)],'Ydata',[y(i,1) y(i,2)]);
    set(rim2,'Xdata',[x(i,2) x(i,3)],'Ydata',[y(i,2) y(i,3)]);
    set(rim3,'Xdata',[x(i,3) x(i,4)],'Ydata',[y(i,3) y(i,4)]);
    set(rim4,'Xdata',[x(i,4) x(i,5)],'Ydata',[y(i,4) y(i,5)]);
    set(rim5,'Xdata',[x(i,5) x(i,6)],'Ydata',[y(i,5) y(i,6)]);
    set(rim6,'Xdata',[x(i,6) x(i,7)],'Ydata',[y(i,6) y(i,7)]);
    set(rim7,'Xdata',[x(i,7) x(i,8)],'Ydata',[y(i,7) y(i,8)]);
    set(rim8,'Xdata',[x(i,8) x(i,1)],'Ydata',[y(i,8) y(i,1)]);
    
    set(inner1,'Xdata',[x(i,9)  x(i,14)],'Ydata', [y(i,9)  y(i,14)]);
    set(inner2,'Xdata',[x(i,10) x(i,15)],'Ydata', [y(i,10) y(i,15)]);
    set(inner3,'Xdata',[x(i,11) x(i,16)],'Ydata', [y(i,11) y(i,16)]);
    set(inner4,'Xdata',[x(i,12) x(i,17)],'Ydata', [y(i,12) y(i,17)]);
    set(inner5,'Xdata',[x(i,13) x(i,18)],'Ydata', [y(i,13) y(i,18)]);    
    set(inner_cir,'Xdata',[0.85*cos(0:pi/20:2*pi)+x0(i)],'Ydata',[0.85*sin(0:pi/20:2*pi)+y0(i)]);

    title(sprintf('Time = %f[sec]',Time(i)),'Color','k');
    drawnow;
    % movie(ii) = getframe(gcf);
    % ii = ii+1;
end

% v = VideoWriter('movie.mp4','MPEG-4');
% open(v);
% writeVideo(v,movie);
% close(v);

