%Holomomic constraints
function [N] = get_N(t,q)

    parameter;

    M = get_M(q);
    h = get_h(q);


    J = [ 1 0 R*cos(q(3)) ;
          0 1 -R*sin(q(3))];
    lambda = inv(J*inv(M)*J')*(J*inv(M)*h);
    N = J'*lambda;
    
end