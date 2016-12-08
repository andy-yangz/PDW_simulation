function dq = rimless(t,q)
parameter;

M = get_M(q);
h = get_h(q);
% u = get_u(t,q);
N = get_N(t,q);


dq = zeros(10,1);
dq(1,1) = q(5);
dq(2,1) = q(6);
dq(3,1) = q(7);
dq(4,1) = q(8);
dq(5:8,1) = inv(M)*(N-h);





end
