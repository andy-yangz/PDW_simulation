function u = get_u(t,q)
parameter;
global tttt;
global uuuu;
global iiii;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%DODC
% T_set = 0.8;
% 
% if rem(t,T_set)<T_set/2
%     v = 4*alpha1/T_set/T_set;
%     u = (I0+m*L*L)*v-m*g*L*sin(q(3));
% else
%     v = -4*alpha1/T_set/T_set;
%     u = (I0+m*L*L)*v-m*g*L*sin(q(3));
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CODC
%     T_set = 0.8;
%     tt = rem(t,T_set);
%     v0 = 6*alpha1/T_set/T_set;
%     v = -2*v0*tt/T_set + v0;
%     u = (I0+m*L*L)*v-m*g*L*sin(q(3));

%.........................................

    T_set = 0.8;
    tt = rem(t,T_set);
    v0 = alpha1*pi*pi/2/T_set/T_set;
    v = v0*cos(pi*tt/T_set);
    u = (I0+m*L*L)*v-m*g*L*sin(q(3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M = [I0+m*L^2 0;
%     0 I1];
% h = [-m*g*L*sin(q(3));
%     0];
% fai = q(3)-q(4);
% fai1= q(7)-q(8);
% 
% fai_d = 8*pi;
% 
% A = [-q(8)/fai1/fai1/fai1 q(7)/fai1/fai1/fai1]*inv(M)*[1;-1];
% B = [-q(8)/fai1/fai1/fai1 q(7)/fai1/fai1/fai1]*inv(M)*h;
% v = 120*alpha1*(fai^3)/((fai_d)^5)-180*alpha1*fai*fai/((fai_d)^4)+60*alpha1*fai/((fai_d)^3);
% u=(v+B)/A;



uuuu(iiii)=u;
tttt(iiii)=t;
iiii= iiii+1;
end

