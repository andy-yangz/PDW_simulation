function [value, isterminal, direction] = collision(t,q)

    parameter;
    
    value(1) = simulation_time-t;
    %disp(sprintf('value = %f',value));
    isterminal(1) = 1;        
    direction = -1;
    
end