classdef SliderPlant<handle
	% Slider Class, method and properties of it

	properties (SetAccess = private)
		% Physical parameter of slider
		m1 = 1.0; 
		m2 = 1.0;
		a = 0.3;
		b = 0.2;
		R = 1.0243;
		I1 = 0.1; 
		I2 = 0.1;
		%Control parameter
		S = [0 0 1 -1]';

	end
	properties (Constant)
		g = 9.81; 
	end
	properties (Dependent)
		M % Inertial matrix
		h 
		N % Right side of equation
		u % Control input
		y % Control output
		dy

	end
	properties
		q0 = [0 0 pi/6 0 0 0 0 0]';
		q = [0 0 pi/6 0 0 0 0 0]'; % x,z,th1,th2
		t = 0;
		simulation_time = 5;
		T; % Time record
		Q; % State record
	end

	methods
		function obj=SliderPlant(q, simulation_time)
			% Constructor, you can set initil q and simulation times
			if (nargin>0)
				obj.q0 = q;
				obj.q = q;
				obj.simulation_time = simulation_time;
			end
		end


		function M = get.M(obj)
			M = zeros(4,4);
			M(1,1) = obj.m1+obj.m2;
			M(1,2) = 0;
			M(1,3) = obj.a*(obj.m1+obj.m2)*cos(obj.q(3));
			M(1,4) = -obj.b*obj.m2*cos(obj.q(4));
			M(2,1) = 0;
			M(2,2) = obj.m1+obj.m2;
			M(2,3) = -obj.a*(obj.m1+obj.m2)*sin(obj.q(3));
			M(2,4) = obj.b*obj.m2*sin(obj.q(4));
			M(3,1) = obj.a*(obj.m1+obj.m2)*cos(obj.q(3));
			M(3,2) = -obj.a*(obj.m1+obj.m2)*sin(obj.q(3));
			M(3,3) = obj.I1+obj.a^2*(obj.m1+obj.m2);
			M(3,4) = -obj.a*obj.b*obj.m2*cos(obj.q(3)-obj.q(4));
			M(4,1) = -obj.b*obj.m2*cos(obj.q(4));
			M(4,2) = obj.b*obj.m2*sin(obj.q(4));
			M(4,3) = -obj.a*obj.b*obj.m2*cos(obj.q(3)-obj.q(4));
			M(4,4) = obj.I2+obj.b^2*obj.m2;
		end

		function h = get.h(obj)

			h = zeros(4,1);
			h(1,1) = -obj.a*obj.q(7)^2*(obj.m1+obj.m2)*sin(obj.q(3)) + obj.b*obj.q(8)^2*obj.m2*sin(obj.q(4));
			h(2,1) = (obj.m1+obj.m2)*(obj.g-obj.a*obj.q(7)^2*cos(obj.q(3))) + obj.b*obj.q(8)^2*obj.m2*cos(obj.q(4));
			h(3,1) = -obj.a*obj.g*(obj.m1+obj.m2)*sin(obj.q(3))+obj.b*obj.q(7)^2*obj.m2*sin(obj.q(3)-obj.q(4));
			h(4,1) = obj.b*obj.m2*(obj.a*obj.q(7)^2*sin(obj.q(3)-obj.q(4)) + obj.g*sin(obj.q(4)));

		end

		function N = get.N(obj)

		    J = [ 1 0 obj.R*(cos(obj.q(3))-1) 0;
		          0 1 -obj.R*sin(obj.q(3)) 0];
		    J_dot = [0 0 -obj.R*sin(obj.q(3))*obj.q(7) 0;
		    		0 0 -obj.R*cos(obj.q(3))*obj.q(7) 0];
		    lambda = inv(J*inv(obj.M)*J')*(J*inv(obj.M)*(obj.h-obj.S*obj.u)-J_dot*obj.q(5:8));
		    N = J'*lambda + obj.S*obj.u;
		end

		function y = get.y(obj)
			y = obj.q(3) - obj.q(4);
		end

		function dy = get.dy(obj)
			dy = obj.q(7) - obj.q(8);
		end

		function u = get.u(obj)
			% Input parameters
			J = [ 1 0 obj.R*(cos(obj.q(3))-1) 0;
		          0 1 -obj.R*sin(obj.q(3)) 0];
		    J_dot = [0 0 -obj.R*sin(obj.q(3))*obj.q(7) 0;
		    		0 0 -obj.R*cos(obj.q(3))*obj.q(7) 0];
			Am = 0.8; %Amplitude
			T = 0.5; %Periodic time
			omega = 2*pi/T;
			yd = Am*sin(omega*obj.t);
			dyd = Am*omega*cos(omega*obj.t);
			ddyd = -Am*omega^2*sin(omega*obj.t);
			Kp = 20;
			Kd = 10;
			v = ddyd+Kd*(dyd-obj.dy)+Kp*(yd-obj.y);
			X = J*inv(obj.M)*J';
			Y = (eye(4)-J'*inv(X)*J*inv(obj.M));
			A = obj.S'*inv(obj.M)*Y*obj.S;
			B = obj.S'*inv(obj.M)*(Y*obj.h+J'*inv(X)*J_dot*obj.q(5:8));
			u = inv(A)*(v+B);
			% u = 4*sin(obj.t*pi);
		end
		function dq = slider_dynamic(obj,t,q)
			obj.q = q;
			obj.t = t;
			dq = zeros(8,1);
			dq(1:4,1) = q(5:8);
			dq(5:8,1) = inv(obj.M)*(obj.N-obj.h);
		end

		function simulate(obj, q, simulation_time)
			if (nargin>1)
				obj.q = q;
				obj.simulation_time = simulation_time;
			end
			options = odeset('RelTol',1e-12,'AbsTol',1e-12*ones(1,8),'Refine',15);
			[T,Q] = ode45(@obj.slider_dynamic,[0 obj.simulation_time],obj.q,options);
			ii = 1;
			for i = 1:length(T) % Select the appropriate time
			    if T(i) >= 0.001*(ii-1)
			        Time(ii) = T(i);
			        Result(ii,:) = Q(i,:); 
			        ii = ii + 1;
			    end
			end
			obj.T = Time;

			obj.Q = Result;
		end

		function playback(obj)
			ini_angle = pi/4;
			len = length(obj.T);
			x=zeros(len,6); 
			% x, x1 upper center of circle, x2 below COC, xc contact point, x_bar center,x2 little mass
			y=zeros(len,6);
			angles = zeros(len,4);
			for i=1:len
			    x(i,1) = obj.Q(i,1);
			    y(i,1) = obj.Q(i,2);
			    x(i,2) = x(i,1)+obj.R*sin(obj.Q(i,3)); % upper COC
			    y(i,2) = y(i,1)+obj.R*cos(obj.Q(i,3));
			    x(i,3) = x(i,2)-(2*obj.R-2*obj.a)*sin(obj.Q(i,3)); % Below COC
			    y(i,3) = y(i,2)-(2*obj.R-2*obj.a)*cos(obj.Q(i,3));
			    x(i,4) = x(i,2); % Contact point
			    y(i,4) = y(i,2)-obj.R;
			    x(i,5) = x(i,1)+obj.a*sin(obj.Q(i,3));
			    y(i,5) = y(i,1)+obj.a*cos(obj.Q(i,3));
			    x(i,6) = x(i,5)-obj.b*sin(obj.Q(i,4));
			    y(i,6) = y(i,5)-obj.b*cos(obj.Q(i,4));

			    angles(i,1) = 5*pi/4-obj.Q(i,3); % Start angle of below arc 
			    angles(i,2) = angles(i,1)+pi/2; % End angle
			    angles(i,3) = angles(i,1)-pi;
			    angles(i,4) = angles(i,2)-pi;
			end

			figure(100)	
			t1 = linspace(angles(1,1),angles(1,2));
			x1 = obj.R*cos(t1) + x(1,2);
			y1 = obj.R*sin(t1) + y(1,2);
			t2 = linspace(angles(1,3),angles(1,4));
			x2 = obj.R*cos(t2) + x(1,3);
			y2 = obj.R*sin(t2) + y(1,3);
			ground = plot([obj.q0(1)-5, obj.q0(1)+5],[obj.q0(2)-obj.R*(1-cos(obj.q0(3))),obj.q0(2)-obj.R*(1-cos(obj.q0(3)))],'k','LineWidth',2);hold on;
			below_arc = fill(x1,y1,'y');hold on;
			upper_arc = fill(x2,y2,'y'); hold on;
			contact_point = plot(x(1,4),y(1,4),'ro','MarkerFaceColor','r');
			mass_bar = plot(x(1,5:6),y(1,5:6),'b-','LineWidth',2);
			mass = plot(x(1,6),y(1,6),'ko','MarkerSize',6,'MarkerFaceColor','k');

			axis equal;
			axis([obj.q0(1)-2, obj.q0(1)+2,obj.q0(2)-1, obj.q0(2)+2]);
			ax = gca;
			ax.SortMethod = 'depth';
			ii = 1;
			for i=1:15:len
			    set(gcf,'Color','k');
			    t1 = linspace(angles(i,1),angles(i,2));
			    x1 = obj.R*cos(t1) + x(i,2);
			    y1 = obj.R*sin(t1) + y(i,2);
			    t2 = linspace(angles(i,3),angles(i,4));
			    x2 = obj.R*cos(t2) + x(i,3);
			    y2 = obj.R*sin(t2) + y(i,3);

			    set(upper_arc,'Vertices',[x1(:),y1(:)]);
			    set(below_arc,'Vertices',[x2(:),y2(:)]);
			    set(contact_point,'Xdata',x(i,4),'Ydata',y(i,4));
			    set(mass_bar,'Xdata', x(i,5:6), 'Ydata', y(i,5:6));
			    set(mass, 'Xdata', x(i,6), 'Ydata', y(i,6));
			    title(sprintf('Time = %f[sec]',obj.T(i)),'Color','w');
			    drawnow;

			    % movie(ii) = getframe(gcf);
			    % if ii == 1 
			    %     [mov(:,:,1,ii), map] = rgb2ind( movie(ii).cdata, 256, 'nodither');
			    % else
			    %     mov(:,:,1,ii) = rgb2ind( movie(ii).cdata, map, 'nodither');
			    % end
			    % ii = ii+1;
			end
		end

	end

end
