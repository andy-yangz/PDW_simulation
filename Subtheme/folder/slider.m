function dq = slider(t,q)
parameter;

M = get_M(q);
h = get_h(q);
N = get_N(q);

dq = zeros(8,1);
dq(1:4,1) = q(5:8);
dq(5:8,1) = inv(M)*(N-h);
end

% function h = get_h(q)
% 	parameter;
% 	h = zeros(3,1);
% 	h(1,1) = -a*q(6)^2*m1*sin(q(3));
% 	h(2,1) = m1*(g-a*q(6)^2*cos(q(3)));
% 	h(3,1) = -a*g*m1*sin(q(3));

% end

% function M = get_M(q)
% 	parameter;
% 	M = zeros(3,3);
% 	M(1,1) = m1;
% 	M(1,2) = 0;
% 	M(1,3) = a*m1*cos(q(3));
% 	M(2,1) = 0;
% 	M(2,2) = m1;
% 	M(2,3) = -a*m1*sin(q(3));
% 	M(3,1) = a*m1*cos(q(3));
% 	M(3,2) = -a*m1*sin(q(3));
% 	M(3,3) = I1+a^2*m1;

% end

% function N = get_N(q,M,h)
% 	parameter;
%     J = [ 1 0 R*(cos(q(3))-1);
%           0 1	-R*sin(q(3)) ];
%     lambda = inv(J*inv(M)*J')*(J*inv(M)*h);
%     N = J'*lambda;
    
% end