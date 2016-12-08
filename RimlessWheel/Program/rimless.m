function dq = rimless(t,q)
parameter;

M = get_M(q);
h = get_h(q);

N = get_N(t,q);

dq = zeros(6,1);
dq(1,1) = q(4);
dq(2,1) = q(5);
dq(3,1) = q(6);
dq(4:6,1) = inv(M)*(N-h);

end
