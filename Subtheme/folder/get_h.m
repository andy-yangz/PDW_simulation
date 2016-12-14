
function h = get_h(q)

parameter;

h = zeros(3,1);
h(1,1) = -a*q(7)^2*(m1+m2)*sin(q(3)) + b*q(8)^2*m2*sin(q(4));
h(2,1) = (m1+m2)*(g-a*q(7)^2*cos(q(3))) + b*q(8)^2*m2*cos(q(4));
h(3,1) = -a*g*(m1+m2)*sin(q(3))+b*q(7)^2*m2*sin(q(7)-q(8));
h(4,1) = b*m2*(a*q(7)^2*sin(q(7)-q(8)) + g*sin(q(8)));

end