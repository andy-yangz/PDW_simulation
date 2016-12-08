
clear all;
close all;
clc
parameter;

time = [];
result = [];

tic;

tspan = [0 2]; % Time span of this simulation, first factor is current time, and later one is object time
options = odeset('Events',@collision,'RelTol',1e-12,'AbsTol',1e-12*ones(1,6),'Refine',15);
% options = odeset('Events',@collision,'RelTol',1e-5);
%%

for step_number = 1:100
    if tspan(1)>=tspan(2)  % When current time equal to object time, simulation finish
        disp('Simulation finished!');
        break;
    elseif tspan(1)~=0
        disp(sprintf('impacttime=%f',time(end)));
    end
    
    [T,Q] = ode45(@rimless,tspan,q0,options);
    nt = length(T); % How many time calculated
    tspan(1) = T(end); % Mark time
    time=[time;T]; % Put time calculated into the time array
    result=[result;Q]; % Put calculated result into the result array
       
    q0 = change(Q(nt,:));   % Q(nt) is the last term of condition claculated, which means the condition just before collision
    
end

%% PLot several figures we need
figure(1)
plot(time,result(:,3));
xlabel('Time [s]');
ylabel('Angular position [rad]');
grid on;

figure(2)
plot(time,result(:,6));
xlabel('Time [s]');
ylabel('Angular velocity [rad]');
grid on;

figure(3)
plot(result(:,3),result(:,6));
xlabel('th1 [rad]');
ylabel('dth1 [rad/s]');
grid on;

%% Assure the least time of a calculated period
ii = 1;
for i = 1:length(time)
    if time(i) >= 0.001*(ii-1)
        Time(ii) = time(i);
        Result(ii,:) = result(i,:);
        ii = ii + 1;
    end
end

Time = Time(1:10:end);
Result = Result(1:10:end,:);

% ii = 1;
% for i = 1:10:length(time)
%     Time(ii) = time(i);
%     Result(ii,:) = result(i,:);
%     ii = ii + 1;
% end
   
x=zeros(length(Time),8);
y=zeros(length(Time),8);
for i=1:length(Time)
    x0(i) = Result(i,1)+L*sin(Result(i,3));  % x position of central of rimless wheel
    x(i,1) = Result(i,1); % The stance leg x position
    x(i,2) = x0(i)+L*sin(alpha1-Result(i,3)); 
    x(i,3) = x0(i)+L*sin(2*alpha1-Result(i,3));
    x(i,4) = x0(i)+L*sin(3*alpha1-Result(i,3));
    x(i,5) = x0(i)+L*sin(4*alpha1-Result(i,3));
    x(i,6) = x0(i)-L*sin(3*alpha1+Result(i,3));
    x(i,7) = x0(i)-L*sin(2*alpha1+Result(i,3));
    x(i,8) = x0(i)-L*sin(alpha1+Result(i,3));
    
    y0(i) = Result(i,2)+L*cos(Result(i,3));
    y(i,1) = Result(i,2);
    y(i,2) = y0(i)-L*cos(alpha1-Result(i,3));
    y(i,3) = y0(i)-L*cos(2*alpha1-Result(i,3));
    y(i,4) = y0(i)-L*cos(3*alpha1-Result(i,3));
    y(i,5) = y0(i)-L*cos(4*alpha1-Result(i,3));
    y(i,6) = y0(i)-L*cos(3*alpha1+Result(i,3));
    y(i,7) = y0(i)-L*cos(2*alpha1+Result(i,3));
    y(i,8) = y0(i)-L*cos(alpha1+Result(i,3));
end
figure(4)

slope_line = plot([-(x0(1)+5)*cos(phi) (x0(end)+5)*cos(phi)],[(x0(1)+5)*sin(phi) -(x0(end)+5)*sin(phi)],'k','linewidth',2);hold on; % Draw the slope
stance_leg = plot([x(1,1) x0(1)],[y(1,1) y0(1)],'r','linewidth',2.5);hold on; % Draw stance leg
leg_26 = plot([x(1,2) x(1,6)],[y(1,2) y(1,6)],'b','linewidth',2);hold on; % Draw 2 and 6 legs
leg_37 = plot([x(1,3) x(1,7)],[y(1,3) y(1,7)],'b','linewidth',2);hold on; % Draw 3 and 7 legs
leg_48 = plot([x(1,4) x(1,8)],[y(1,4) y(1,8)],'b','linewidth',2);hold on; % Draw 4 and 8 legs
leg_5 = plot([x(1,5) x0(1)],[y(1,5) y0(1)],'b','linewidth',2);hold on;  % Draw 5 leg

axis equal;
 % set(gca,'drawmode','fast');
ax = gca;
% ax.SortMethod = 'childorder';
ax.SortMethod = 'depth';
ii = 1;
for i=1:2:length(Time)
    axis([x0(i)-2.0,x0(i)+2.0,-x0(i)*tan(phi)-0.6,-x0(i)*tan(phi)+2.4]);
    set(gcf,'Color','k');
    set(stance_leg,'Xdata',[x(i,1) x0(i)],'Ydata',[y(i,1) y0(i)]); % Update the animation of rimlesswheel
    set(leg_26,'Xdata',[x(i,2) x(i,6)],'Ydata',[y(i,2) y(i,6)]);
    set(leg_37,'Xdata',[x(i,3) x(i,7)],'Ydata',[y(i,3) y(i,7)]);
    set(leg_48,'Xdata',[x(i,4) x(i,8)],'Ydata',[y(i,4) y(i,8)]);
    set(leg_5,'Xdata',[x(i,5) x0(i)],'Ydata',[y(i,5) y0(i)]);

    title(sprintf('Time = %f[sec]',Time(i)),'Color','w');
    drawnow;
    % movie(ii) = getframe(gcf);

    % For gif
    % f = getframe(gcf);


    % if ii == 1 
    %     [mov(:,:,1,ii), map] = rgb2ind(f.cdata, 256, 'nodither');
    % else
    %     mov(:,:,1,ii) = rgb2ind(f.cdata, map, 'nodither');
    % end
    % ii = ii+1;
end
fprintf('It cost %d seconds.\n', toc);


%avi movie
% movie2avi(mov, 'animation.avi')

%mp4 compressed movie
% v = VideoWriter('movie.mp4','MPEG-4');
% open(v);
% writeVideo(v,movie);
% close(v);

% Create animated GIF
% imwrite(mov, map, 'animation.gif', 'Delaytime',0, 'LoopCount', inf);