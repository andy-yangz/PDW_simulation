L = 1;
R =0.25;
x = [0;0];
x1 = [L*sin(pi/8); L*cos(pi/8)]; 
x2 = [2*x1(1); 0];
xc = [x2(1), x2(2)+0.25];
angle = [3*pi/2; 11*pi/6];

t1 = linspace(angle(1),angle(2));
x_arc = R*cos(t1) + xc(1);
y_arc = R*sin(t1) + xc(2);

pi
plot([x(1), x1(1)],[x(2),x1(2)],'k-','LineWidth',2);hold on;
plot([x1(1),x2(1)],[x1(2),x2(2)],'k-','LineWidth',2);
fill(x_arc, y_arc,'k');

L1 = norm([x_arc(end)-x1(1), y_arc(end)-x1(2)]);
L2 = 0.25;

alpha2 = acos((L^2 + L1^2 - L2^2)/(2*L*L1))
plot([x_arc(end),x1(1)], [y_arc(end),x1(2)])
