function q0 = change(Q)
parameter;

% Change legs   

    Ji = [ 1 0 L*cos(Q(3))-L*cos(alpha1-Q(3)) ;
           0 1 -L*sin(Q(3))-L*sin(alpha1-Q(3)) ];

      
    M = get_M(Q);
    Qp = (eye(3) - inv(M)*Ji' * inv(Ji*inv(M) * Ji') * Ji ) * Q(4:6)'; % Qplus 
    
    q0(1) = Q(1)+L*sin(Q(3))+L*sin(alpha1-Q(3)); % Change legs
    q0(2) = Q(2)+L*cos(Q(3))-L*cos(alpha1-Q(3)); % Change stance leg to swing leg
    q0(3) = Q(3)-alpha1; % Positive direction is counterclockwise, 2theta = alpha
    q0(4) = 0;
    q0(5) = 0;
    q0(6) = Qp(3);
        
end

