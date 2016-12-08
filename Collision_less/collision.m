function [value, isterminal, direction] = collision(t,q)

    parameter;
    
    value(1) = L*cos(q(3)) - L*cos(alpha1-q(3));


    %disp(sprintf('value = %f',value));
    isterminal(1) = 1;        
    direction = -1;
    
end