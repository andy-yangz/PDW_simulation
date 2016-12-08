%Šµ«s—ñ‚ðŠl“¾‚·‚éŠÖ”
function M = get_M(q)

parameter;

% M = zeros(4,4);
% M(1,1) = m;
% M(1,2) = 0;
% M(1,3) = m*L*cos(q(3));
% M(1,4) = 0;
% M(2,1) = 0;
% M(2,2) = m;
% M(2,3) = -L*m*sin(q(3));
% M(2,4) = 0;
% M(3,1) = L*m*cos(q(3));
% M(3,2) = -L*m*sin(q(3));
% M(3,3) = I0+m*L^2;
% M(3,4) = 0;
% M(4,4) = I1;

M = [m, 0, m*L*cos(q(3)),0;
	 0, m, -m*L*sin(q(3)), 0;
	 L*m*cos(q(3), -L*m*sin(q(3))), I0+m*L^2, 0;
	 0, 0, 0, I1]

end