function [distance, isterminal, direction] = collision(t,q)

    parameter;
    
    distance(1) = L*cos(q(3)) + 2*L*sin(alpha1/2)*sin(phi) - L*cos(alpha1-q(3)); % The distance between forward leg and the slope, in slope vertical direction
    %disp(sprintf('distance = %f',distance));
    isterminal(1) = 1;    % When event happen halt integration    
    direction = -1;		% Zero can only approach from decreasing
    
end