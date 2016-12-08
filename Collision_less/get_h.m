%コリオリ力・中心力・重力項を獲得する関数
function h = get_h(q)

parameter;

h = zeros(4,1);
h(1,1) = -q(7)^2*L*m*sin(q(3));
h(2,1) = m*(g-q(7)^2*L*cos(q(3)));
h(3,1) = -m*g*L*sin(q(3));

end