%äµê´çsóÒÇälìæÇ∑ÇÈä÷êî
function M = get_M(q)

parameter;

M = zeros(4,4);
M(1,1) = m1+m2;
M(1,2) = 0;
M(1,3) = a*(m1+m2)*cos(q(3));
M(1,4) = -b*m2*cos(q(4));
M(2,1) = 0;
M(2,2) = m1+m2;
M(2,3) = -a*(m1+m2)*sin(q(3));
M(2,4) = b*m2*sin(q(4));
M(3,1) = a*(m1+m2)*cos(q(3));
M(3,2) = -a*(m1+m2)*sin(q(3));
M(3,3) = I1+a^2*(m1+m2);
M(3,4) = -a*b*m2*cos(q(3)-q(4));
M(4,1) = -b*m2*cos(q(4));
M(4,2) = b*m2*sin(q(4));
M(4,3) = -a*b*m2*cos(q(3)-q(4));
M(4,4) = I2+b^2*m2;
end