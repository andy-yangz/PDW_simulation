% ƒzƒƒmƒ~ƒbƒNS‘©—Í‚ğŠl“¾‚·‚éŠÖ”
function N = get_N(q)

    parameter;

    M = get_M(q);
    h = get_h(q);


    J = [ 1 0 R*(cos(q(3))-1) 0;
          0 1 -R*sin(q(3)) 0];
    J_dot = [0 0 -R*sin(q(3))*q(7) 0;
    		0 0 -R*cos(q(3))*q(7) 0];
    lambda = inv(J*inv(M)*J')*(J*inv(M)*h-J_dot*q(5:8));
    N = J'*lambda;
end