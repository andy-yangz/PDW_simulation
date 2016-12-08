%Holomomic constraints
function [N] = get_N(t,q)

    parameter;

    M = get_M(q);
    h = get_h(q);


    J = [ 1 0 0 ;
          0 1 0 ];
    lambda = inv(J*inv(M)*J')*(J*inv(M)*h);
    N = J'*lambda;
    
end