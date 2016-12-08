
clear all;
close all;
clc

% you can also initiliaze SliderPlant in command window
% you can initialize q0 and simulation_time for it
% r = SliderPlant;
r = SliderPlant([0 0 0 pi/3 0 0 0 0],5);
time = [];
result = [];
q0 = r.q;
tspan = [0 15];
% options = odeset('Events',@collision,'RelTol',1e-12,'AbsTol',1e-12*ones(1,8),'Refine',15);
% options = odeset('Events',@collision,'RelTol',1e-5);
options = odeset('RelTol',1e-12,'AbsTol',1e-12*ones(1,8),'Refine',15);
%%
% tic;
[T,Q] = ode45(@r.slider_dynamic,[0 r.simulation_time],r.q,options);
time=[time;T];
result=[result;Q];
% fprintf('%d sec cost.',toc);

%
% figure(1)
% plot(time,result(:,1));
% xlabel('Time [s]');
% ylabel('x [m]');
% grid on;

% figure(2)
% plot(time,result(:,2));
% xlabel('Time [s]');
% ylabel('z [m]');
% grid on;

% figure(3)
% plot(time,result(:,3));
% xlabel('Time [s]');
% ylabel('\theta1 [m]');
% grid on;

% figure(4)
% plot(time,result(:,4));
% xlabel('Time [s]');
% ylabel('\theta2 [m]');
% grid on;

% figure(3)
% plot(result(:,3),result(:,6));
% xlabel('th1 [rad]');
% ylabel('dth1 [rad/s]');
% grid on;


ii = 1;
for i = 1:length(time)
    if time(i) >= 0.001*(ii-1)
        Time(ii) = time(i);
        Result(ii,:) = result(i,:); %Result x z theta
        ii = ii + 1;
    end
end

% L = sqrt(2*R*a-a^2);
% ini_angle = asin(L/R);
ini_angle = pi/4;
x=zeros(length(Time),6); 
% x, x1 upper center of circle, x2 below COC, xc contact point, x_bar center,x2 little mass
y=zeros(length(Time),6);
angles = zeros(length(Time),4);
for i=1:length(Time)


    x(i,1) = Result(i,1);
    y(i,1) = Result(i,2);
    x(i,2) = x(i,1)+r.R*sin(Result(i,3)); % upper COC
    y(i,2) = y(i,1)+r.R*cos(Result(i,3));
    x(i,3) = x(i,2)-(2*r.R-2*r.a)*sin(Result(i,3)); % Below COC
    y(i,3) = y(i,2)-(2*r.R-2*r.a)*cos(Result(i,3));
    x(i,4) = x(i,2); % Contact point
    y(i,4) = y(i,2)-r.R;
    x(i,5) = x(i,1)+r.a*sin(Result(i,3));
    y(i,5) = y(i,1)+r.a*cos(Result(i,3));
    x(i,6) = x(i,5)-r.b*sin(Result(i,4));
    y(i,6) = y(i,5)-r.b*cos(Result(i,4));

    angles(i,1) = 5*pi/4-Result(i,3); % Start angle of below arc 
    angles(i,2) = angles(i,1)+pi/2; % End angle
    angles(i,3) = angles(i,1)-pi;
    angles(i,4) = angles(i,2)-pi;
end

figure(4)
t1 = linspace(angles(1,1),angles(1,2));
x1 = r.R*cos(t1) + x(1,2);
y1 = r.R*sin(t1) + y(1,2);
t2 = linspace(angles(1,3),angles(1,4));
x2 = r.R*cos(t2) + x(1,3);
y2 = r.R*sin(t2) + y(1,3);
ground = plot([q0(1)-5, q0(1)+5],[q0(2)-r.R*(1-cos(q0(3))),q0(2)-r.R*(1-cos(q0(3)))],'k','LineWidth',2);hold on;

% below_arc = plot_arc(angles(1,1),angles(1,2),x(i,2),y(i,2),R);hold on;
% upper_arc = plot_arc(angles(1,3),angles(1,4),x(i,3),y(i,3),R);
below_arc = fill(x1,y1,'y');hold on;
upper_arc = fill(x2,y2,'y'); hold on;
contact_point = plot(x(1,4),y(1,4),'ro','MarkerFaceColor','r');
mass_bar = plot(x(1,5:6),y(1,5:6),'b-','LineWidth',2);
mass = plot(x(1,6),y(1,6),'ko','MarkerSize',6,'MarkerFaceColor','k');

axis equal;
axis([q0(1)-2, q0(1)+2,q0(2)-1, q0(2)+2]);
ax = gca;
ax.SortMethod = 'depth';
ii = 1;
for i=1:10:length(Time)
    set(gcf,'Color','k');
    t1 = linspace(angles(i,1),angles(i,2));
    x1 = r.R*cos(t1) + x(i,2);
    y1 = r.R*sin(t1) + y(i,2);
    t2 = linspace(angles(i,3),angles(i,4));
    x2 = r.R*cos(t2) + x(i,3);
    y2 = r.R*sin(t2) + y(i,3);

    set(upper_arc,'Vertices',[x1(:),y1(:)]);
    set(below_arc,'Vertices',[x2(:),y2(:)]);
    set(contact_point,'Xdata',x(i,4),'Ydata',y(i,4));
    set(mass_bar,'Xdata', x(i,5:6), 'Ydata', y(i,5:6));
    set(mass, 'Xdata', x(i,6), 'Ydata', y(i,6));
    title(sprintf('Time = %f[sec]',Time(i)),'Color','w');
    drawnow;

    % movie(ii) = getframe(gcf);
    % if ii == 1 
    %     [mov(:,:,1,ii), map] = rgb2ind( movie(ii).cdata, 256, 'nodither');
    % else
    %     mov(:,:,1,ii) = rgb2ind( movie(ii).cdata, map, 'nodither');
    % end
    % ii = ii+1;
end


% Create animated GIF
% imwrite(mov, map, 'animation.gif', 'Delaytime',0, 'LoopCount', inf);
%mp4 compressed movie
% v = VideoWriter('movie.mp4','MPEG-4');
% open(v);
% writeVideo(v,movie);
% close(v);

%AVI file using Motion JPEG encoding
% v = VideoWriter('movie.avi','Motion JPEG AVI');
% open(v);
% writeVideo(v,movie)
% close(v);


