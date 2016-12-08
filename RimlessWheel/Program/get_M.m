%Šµ�?«�?s—ñ‚ðŠl“¾‚·‚éŠÖ�?�?
function M = get_M(q)

parameter;

M = zeros(3,3); 
M(1,1) = m;
M(1,2) = 0;
M(1,3) = m*L*cos(q(3)); % q3 is theta pitch angle
M(2,1) = 0;
M(2,2) = m;
M(2,3) = -L*m*sin(q(3));
M(3,1) = L*m*cos(q(3));
M(3,2) = -L*m*sin(q(3));
M(3,3) = I0+m*L^2;

end