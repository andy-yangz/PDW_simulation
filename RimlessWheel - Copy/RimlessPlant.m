classdef RimlessPlant<handle
	
	properties (SetAccess = private)
		m = 1.0; % Mass
		I = 0.1; % Moment of Inertia
        L = 1;
		alpha1 = (45/180)*pi;
		phi = 0.1;
		q0 = [0 0 0.1 0 0 0];
		q = [0 0 0.1 0 0 0];
		simulation_time = 5;
	end
	properties (Constant)
		g = 9.81;
	end
	properties (Dependent)
		M
		h
		N
	end
	properties
		T;
		Q;
	end

	methods
		function obj = RimlessPlant(q, simulation_time)
			if (nargin>0)
				obj.q0 = q;
				obj.q = q;
				obj.simulation_time = simulation_time;
			end
		end

		function M = get.M(obj)
			M = zeros(3,3); 
			M(1,1) = obj.m;
			M(1,2) = 0;
			M(1,3) = obj.m*obj.L*cos(obj.q(3)); % obj.q3 is theta pitch angle
			M(2,1) = 0;
			M(2,2) = obj.m;
			M(2,3) = -obj.L*obj.m*sin(obj.q(3));
			M(3,1) = obj.L*obj.m*cos(obj.q(3));
			M(3,2) = -obj.L*obj.m*sin(obj.q(3));
			M(3,3) = obj.I+obj.m*obj.L^2;
		end

		function h = get.h(obj)
			h = zeros(3,1);
			h(1,1) = -obj.q(6)^2*obj.L*obj.m*sin(obj.q(3)); % obj.q6 is theta dot
			h(2,1) = obj.m*(obj.g-obj.q(6)^2*obj.L*cos(obj.q(3)));
			h(3,1) = -obj.m*obj.g*obj.L*sin(obj.q(3));

		end

		function N = get.N(obj)

		    J = [ 1 0 0 ;
		          0 1 0];
		    lambda = inv(J*inv(obj.M)*J')*(J*inv(obj.M)*obj.h);
		    N = J'*lambda;
		    
		end

		function dq = rimless_dynamic(obj,t,q)
			obj.q = q;
			% obj.t = t;
			dq = zeros(6,1);
			dq(1:3,1) = q(4:6);
			dq(4:6,1) = inv(obj.M)*(obj.N-obj.h);
		end

		function q0 = change(obj, Q)
		% Change legs   

		    Ji = [ 1 0 obj.L*cos(Q(3))-obj.L*cos(obj.alpha1-Q(3)) ;
		           0 1 -obj.L*sin(Q(3))-obj.L*sin(obj.alpha1-Q(3)) ];
		    Qp = (eye(3) - inv(obj.M)*Ji' * inv(Ji*inv(obj.M) * Ji') * Ji ) * Q(4:6)'; % Qplus 
		    
		    q0(1) = Q(1)+obj.L*sin(Q(3))+obj.L*sin(obj.alpha1-Q(3)); % Change legs
		    q0(2) = Q(2)+obj.L*cos(Q(3))-obj.L*cos(obj.alpha1-Q(3)); % Change stance leg to swing leg
		    q0(3) = Q(3)-obj.alpha1; % Positive direction is counterclockwise, 2theta = alpha
		    q0(4) = 0;
		    q0(5) = 0;
		    q0(6) = Qp(3);
		        
		end



		function simulate(obj, q, simulation_time)
			if (nargin>1)
				obj.q = q;
				obj.simulation_time = simulation_time;
			end
			time = [];
			result = [];
			t_span = [0 obj.simulation_time];
			q0 = obj.q; % set initial q

			options = odeset('Events',@obj.collision,'RelTol',1e-12,'AbsTol',1e-12*ones(1,6),'Refine',15);
			for step_number = 1:10
			    [T,Q] = ode45(@obj.rimless_dynamic, t_span, q0, options);
			    t_span(1) = T(end); % Mark time
			    time=[time;T]; % Put time calculated into the time array
			    result=[result;Q]; % Put calculated result into the result array
			    q0 = obj.change(Q(end,:));   % Q(nt) is the last term of condition claculated, which means the condition just before collision
			    if T(end) == t_span(2) % If to the end of simulation time break
			    	break
			    end
			end
			ii = 1;
			for i = 1:length(time) % Select the appropriate time
			    if time(i) >= 0.001*(ii-1)
			        Time(ii) = time(i);
			        Result(ii,:) = result(i,:); 
			        ii = ii + 1;
			    end
			end
			obj.T = Time;
			obj.Q = Result;
		end

		function [value, isterminal, direction] = collision(obj,t,q)
			% value = obj.L*cos(obj.q(3)) + 2*obj.L*sin(obj.alpha1/2)*sin(obj.phi) - obj.L*cos(obj.alpha1-obj.q(3));
			value = (obj.alpha1/2 + obj.phi) - obj.q(3);
			isterminal = 1;
			direction = -1;	
		end

		function playback(obj)
			len = length(obj.T);
			x=zeros(len,8);
			y=zeros(len,8);
			for i=1:len
			    x0(i) = obj.Q(i,1)+obj.L*sin(obj.Q(i,3));  % x position of central of rimless wheel
			    x(i,1) = obj.Q(i,1); % obj.The stance leg x position
			    x(i,2) = x0(i)+obj.L*sin(obj.alpha1-obj.Q(i,3)); 
			    x(i,3) = x0(i)+obj.L*sin(2*obj.alpha1-obj.Q(i,3));
			    x(i,4) = x0(i)+obj.L*sin(3*obj.alpha1-obj.Q(i,3));
			    x(i,5) = x0(i)+obj.L*sin(4*obj.alpha1-obj.Q(i,3));
			    x(i,6) = x0(i)-obj.L*sin(3*obj.alpha1+obj.Q(i,3));
			    x(i,7) = x0(i)-obj.L*sin(2*obj.alpha1+obj.Q(i,3));
			    x(i,8) = x0(i)-obj.L*sin(obj.alpha1+obj.Q(i,3));
			    
			    y0(i) = obj.Q(i,2)+obj.L*cos(obj.Q(i,3));
			    y(i,1) = obj.Q(i,2);
			    y(i,2) = y0(i)-obj.L*cos(obj.alpha1-obj.Q(i,3));
			    y(i,3) = y0(i)-obj.L*cos(2*obj.alpha1-obj.Q(i,3));
			    y(i,4) = y0(i)-obj.L*cos(3*obj.alpha1-obj.Q(i,3));
			    y(i,5) = y0(i)-obj.L*cos(4*obj.alpha1-obj.Q(i,3));
			    y(i,6) = y0(i)-obj.L*cos(3*obj.alpha1+obj.Q(i,3));
			    y(i,7) = y0(i)-obj.L*cos(2*obj.alpha1+obj.Q(i,3));
			    y(i,8) = y0(i)-obj.L*cos(obj.alpha1+obj.Q(i,3));
			end
			figure(4)

			slope_line = plot([-(x0(1)+5)*cos(obj.phi) (x0(end)+5)*cos(obj.phi)],[(x0(1)+5)*sin(obj.phi) -(x0(end)+5)*sin(obj.phi)],'k','linewidth',2);hold on; % Draw the slope
			stance_leg = plot([x(1,1) x0(1)],[y(1,1) y0(1)],'r','linewidth',2.5);hold on; % Draw stance leg
			leg_26 = plot([x(1,2) x(1,6)],[y(1,2) y(1,6)],'b','linewidth',2);hold on; % Draw 2 and 6 legs
			leg_37 = plot([x(1,3) x(1,7)],[y(1,3) y(1,7)],'b','linewidth',2);hold on; % Draw 3 and 7 legs
			leg_48 = plot([x(1,4) x(1,8)],[y(1,4) y(1,8)],'b','linewidth',2);hold on; % Draw 4 and 8 legs
			leg_5 = plot([x(1,5) x0(1)],[y(1,5) y0(1)],'b','linewidth',2);hold on;  % Draw 5 leg

			axis equal;
			 % set(gca,'drawmode','fast');
			ax = gca;
			% ax.SortMethod = 'childorder';
			ax.SortMethod = 'depth';
			ii = 1;
			for i=1:10:len
			    axis([x0(i)-2.0,x0(i)+2.0,-x0(i)*tan(obj.phi)-0.6,-x0(i)*tan(obj.phi)+2.4]);
			    set(gcf,'Color','k');
			    set(stance_leg,'Xdata',[x(i,1) x0(i)],'Ydata',[y(i,1) y0(i)]); % Update the animation of rimlesswheel
			    set(leg_26,'Xdata',[x(i,2) x(i,6)],'Ydata',[y(i,2) y(i,6)]);
			    set(leg_37,'Xdata',[x(i,3) x(i,7)],'Ydata',[y(i,3) y(i,7)]);
			    set(leg_48,'Xdata',[x(i,4) x(i,8)],'Ydata',[y(i,4) y(i,8)]);
			    set(leg_5,'Xdata',[x(i,5) x0(i)],'Ydata',[y(i,5) y0(i)]);

			    title(sprintf('Time = %f[sec]',obj.T(i)),'Color','w');
			    drawnow;
			end
		end
	end
end