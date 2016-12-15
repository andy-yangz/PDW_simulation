m = 1.0; % Mass
L = 1;   % Length of the leg
I0 = 1.0; % Moment of inertia wheel, mr2
alpha1 = 45/180*pi; % Angle between two legs, there we set 2pi/8, as we have eight legs
% phi = 3/180*pi; % 
phi = 0.1; % Phi is the angle of slope
g = 9.81; % Gravity acceleration
q0 = [0 0 0.1 0 0 0]; % Initial condition x z theta xdot zdot thetadot

