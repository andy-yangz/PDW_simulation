%ƒRƒŠƒIƒŠ—ÍE’†S—ÍEd—Í€‚ðŠl“¾‚·‚éŠÖ”
function h = get_h(q)

parameter;

h = zeros(3,1);
h(1,1) = -q(6)^2*L*m*sin(q(3)); % q6 is theta dot
h(2,1) = m*(g-q(6)^2*L*cos(q(3)));
h(3,1) = -m*g*L*sin(q(3));

end