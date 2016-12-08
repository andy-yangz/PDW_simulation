function q0 = change(Q)
parameter;

%%
%éxéùãråä∑è’ìÀéû
    M = get_M(Q);
    Ji = [ 1 0 L*cos(Q(3))-L*cos(alpha1-Q(3)) 0;
           0 1 -L*sin(Q(3))-L*sin(alpha1-Q(3)) 0];
%        
%     Ji = [ 1 0 L*cos(Q(3))+L*cos(alpha1+Q(3)) 0;
%            0 1 -L*sin(Q(3))+L*sin(alpha1+Q(3)) 0];
%        if Q(7)<0
%            Ji=J2;
%                 Qp = (eye(4) - inv(M)*Ji'*inv(Ji*inv(M)*Ji')*Ji)*Q(5:8)';
%                 q0(1) = Q(1)+L*sin(Q(3))-L*sin(alpha1+Q(3));
%                 q0(2) = Q(2)+L*cos(Q(3))-L*cos(alpha1+Q(3));
%                 q0(3) = Q(3)+alpha1;
%                 q0(4) = Q(4);
%                 q0(5) = 0;
%                 q0(6) = 0;
%                 q0(7) = -Qp(3);
%                 q0(8) = Qp(4);
% 
%        end
%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%               if Q(7)>=0
%                 Ji=J1;
%                 Qp = (eye(4) - inv(M)*Ji'*inv(Ji*inv(M)*Ji')*Ji)*Q(5:8)';
% 
%                 q0(1) = Q(1)+L*sin(Q(3))+L*sin(alpha1-Q(3));
%                 q0(2) = Q(2)+L*cos(Q(3))-L*cos(alpha1-Q(3));
%                 q0(3) = Q(3)-alpha1;
%                 q0(4) = Q(4);
%                 q0(5) = 0;
%                 q0(6) = 0;
%                 q0(7) = Qp(3);
%                 q0(8) = Qp(4);
% 
%                end

      
    %è’ìÀï˚íˆéÆ

    Qp = (eye(4) - inv(M)*Ji'*inv(Ji*inv(M)*Ji')*Ji)*Q(5:8)';
    
    q0(1) = Q(1)+L*sin(Q(3))+L*sin(alpha1-Q(3));
    q0(2) = Q(2)+L*cos(Q(3))-L*cos(alpha1-Q(3));
    q0(3) = Q(3)-alpha1;
    q0(4) = Q(4);
    q0(5) = 0;
    q0(6) = 0;
    q0(7) = Qp(3);
    q0(8) = Qp(4);
    



        
end

