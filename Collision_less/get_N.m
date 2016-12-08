% ƒzƒƒmƒ~ƒbƒNS‘©—Í‚ğŠl“¾‚·‚éŠÖ”
function [N] = get_N(t,q)

    parameter;


    M = get_M(q);
    h = get_h(q);
    u = get_u(t,q);


    J = [ 1 0 0 0;
          0 1 0 0];
    lambda = inv(J*inv(M)*J')*(J*inv(M)*(h-S*u));
    N = J'*lambda + S*u;
    
end