function q0 = change(Q)
parameter;

%%
%x‹rŒğŠ·Õ“Ë

    Ji = [ 1 0 L*cos(Q(3))-L*cos(alpha1-Q(3)) ;
           0 1 -L*sin(Q(3))-L*sin(alpha1-Q(3)) ];

      
    %Õ“Ë•û’ö®
    M = get_M(Q);
    Qp = (eye(3) - inv(M)*Ji'*inv(Ji*inv(M)*Ji')*Ji)*Q(4:6)';
    
    q0(1) = Q(1)+L*sin(Q(3))+L*sin(alpha1-Q(3));
    q0(2) = Q(2)+L*cos(Q(3))-L*cos(alpha1-Q(3));
    q0(3) = Q(3)-alpha1;
    q0(4) = 0;
    q0(5) = 0;
    q0(6) = Qp(3);
        
end

