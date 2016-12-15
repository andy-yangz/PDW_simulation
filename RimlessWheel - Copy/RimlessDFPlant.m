classdef RimlessDFPlant<handle
	
	properties (SetAccess = private)
		m = 1.0; % Mass
		I = 0.1; % Moment of Inertia
        Leg1 = LegPlant; % Stance Leg
        % Leg2 = LegPlant('arc','0.25');
        Leg2 = LegPlant; % Stance Leg
		alpha1 = (45/180)*pi;
		phi = 0.1;
		q0 = [0 0 0.1 0 0 0]';
		q = [0 0 0.1 0 0 0]';
		simulation_time = 5;
	end
	properties (Constant)
		g = 9.81;
	end
	properties (Dependent)
		M
		h
		N
		beta1 % When standstill the angle of stance 
	end
	properties
		T;
		Q;
		collision_times;
	end

	methods
		function obj = RimlessPlant(q, simulation_time)
			if (nargin>0)
				obj.q0 = q;
				obj.q = q;
				obj.simulation_time = simulation_time;
			end
		end

		function beta1 = get.beta1(obj)
			L2 = sqrt(obj.Leg1.L^2 + obj.Leg2.L^2 - 2*obj.Leg1.L*obj.Leg2.L*cos(pi/4));
			beta1 = pi/2 - asin(sin(pi/4)*obj.Leg2.L/L2);
		end

		function M = get.M(obj)
			M = zeros(3,3); 
			M(1,1) = obj.m;
			M(1,2) = 0;
			M(1,3) = obj.m*obj.Leg1.L*cos(obj.q(3)); % obj.q3 is theta pitch angle
			M(2,1) = 0;
			M(2,2) = obj.m;
			M(2,3) = -obj.Leg1.L*obj.m*sin(obj.q(3));
			M(3,1) = obj.Leg1.L*obj.m*cos(obj.q(3));
			M(3,2) = -obj.Leg1.L*obj.m*sin(obj.q(3));
			M(3,3) = obj.I+obj.m*obj.Leg1.L^2;
		end

		function h = get.h(obj)
			h = zeros(3,1);
			h(1,1) = -obj.q(6)^2*obj.Leg1.L*obj.m*sin(obj.q(3)); % obj.q6 is theta dot
			h(2,1) = obj.m*(obj.g-obj.q(6)^2*obj.Leg1.L*cos(obj.q(3)));
			h(3,1) = -obj.m*obj.g*obj.Leg1.L*sin(obj.q(3));

		end

		function N = get.N(obj)
			if obj.Leg1.form == 'point'
			    J = [ 1 0 0 ;
			          0 1 0];
			    J_dot = zeros(2,3);
			elseif obj.Leg1.form == 'arc'
				J = [1 0 obj.Leg1.R*(cos(obj.q(3)+obj.alpha1/2)-1);
					 0 1 -obj.Leg1.R*sin(obj.q(3)+obj.alpha1/2)];
				J_dot = [0 0 -obj.R*sin(obj.q(3)+obj.alpha1/2)*obj.q(6);
						 0 0 -obj.R*cos(obj.q(3)+obj.alpha1/2)*obj.q(6)];
			end

		    lambda = inv(J*inv(obj.M)*J')*(J*inv(obj.M)*obj.h - ...
		    		J_dot*obj.q(4:6));
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

		    Ji = [ 1 0 obj.Leg1.L*cos(Q(3))-obj.Leg2.L*cos(obj.alpha1-Q(3)) ;
		           0 1 -obj.Leg1.L*sin(Q(3))-obj.Leg2.L*sin(obj.alpha1-Q(3)) ];
		    Qp = (eye(3) - inv(obj.M)*Ji' * inv(Ji*inv(obj.M) * Ji') * Ji ) * Q(4:6)'; % Qplus 
		    
		    q0(1) = Q(1)+obj.Leg1.L*sin(Q(3))+obj.Leg2.L*sin(obj.alpha1-Q(3)); % Change legs
		    q0(2) = Q(2)+obj.Leg1.L*cos(Q(3))-obj.Leg2.L*cos(obj.alpha1-Q(3)); % Change stance leg to swing leg
		    q0(3) = Q(3)-obj.alpha1; % Positive direction is counterclockwise, 2theta = alpha
		    q0(4) = 0;
		    q0(5) = 0;
		    q0(6) = Qp(3);
		        
		end

		function [value, isterminal, direction] = collision(obj,t,q)
			  % value = obj.L*cos(obj.q(3)) + 2*obj.L*sin(obj.alpha1/2)*sin(obj.phi) - obj.L*cos(obj.alpha1-obj.q(3));
			value = (obj.beta1 + obj.phi) - obj.q(3);
			isterminal = 1;
			direction = -1;	
		end

		function simulate(obj, q, simulation_time)
			if (nargin>1)
				obj.q = q;
				obj.simulation_time = simulation_time;
			end
			time = [];
			result = [];
			collision_times = [];
			t_span = [0 obj.simulation_time];
			q0 = obj.q; % set initial q

			% Option choose short leg collision event
			options = odeset('Events',@obj.collision,'RelTol',1e-12,'AbsTol',1e-12*ones(1,6),'Refine',15);
			% Option choose 
			% options_long = odeset('Events',@obj.long_leg_collision,'RelTol',1e-12,'AbsTol',1e-12*ones(1,6),'Refine',15);
			
			L0 = [obj.Leg1, obj.Leg2]; % Store initial value 

			for stride_number = 1:100
				% Long leg is stance leg
			    [T,Q] = ode45(@obj.rimless_dynamic, t_span, q0, options);
			    t_span(1) = T(end); % Mark time
			    fprintf('%f sec hit ground.\n',T(end));
			    collision_times = [collision_times;T(end)];
			    time=[time;T]; % Put time calculated into the time array
			    result=[result;Q]; % Put calculated result into the result array
			    q0 = obj.change(Q(end,:));   % Q(nt) is the last term of condition claculated, which means the condition just before collision
			    if T(end) == t_span(2) % If to the end of simulation time break
			    	break
			    end
			    temp = obj.Leg1;
			    obj.Leg1 = obj.Leg2;
			    obj.Leg2 = temp;

			end
			ii = 1;
			for i = 1:length(time) % Select the appropriate time
			    if time(i) >= 0.001*(ii-1) || any(abs(time(i)-collision_times(1:end-1))<1e-10)
			        Time(ii) = time(i);
			        Result(ii,:) = result(i,:); 
			        ii = ii + 1;
			    end
			end

			obj.T = Time;
			obj.Q = Result;
			obj.collision_times = collision_times;
			obj.Leg1 = L0(1);
			obj.Leg2 = L0(2);
		end


		function playback(obj)
			len = length(obj.T);
			x=zeros(len,8);
			y=zeros(len,8);
			x_c = zeros(len,4);
			y_c = zeros(len,4);
			x_f = zeros(100,4,len); % The coordinates of four feet store in a
			y_f = zeros(100,4,len); % 3D matrix

			% l = sqrt(1^2 + 0.25^2 -2*1*0.25*cos(22.5));
			l = 0.775;

			collision_index = 1;
			for i=1:len
				% Change animation when change leg
				if obj.T(i) == obj.collision_times(collision_index)
				    temp = obj.Leg1;
				    obj.Leg1 = obj.Leg2;
				    obj.Leg2 = temp;
				    collision_index = collision_index+1;
				end
				% Get all critical point in rimless wheel
			    x0(i) = obj.Q(i,1)+obj.Leg1.L*sin(obj.Q(i,3));  % x position of central of rimless wheel
			    x(i,1) = obj.Q(i,1); % obj.The stance leg x position
			    x(i,2) = x0(i)+obj.Leg2.L*sin(obj.alpha1-obj.Q(i,3)); 
			    x(i,3) = x0(i)+obj.Leg1.L*sin(2*obj.alpha1-obj.Q(i,3));
			    x(i,4) = x0(i)+obj.Leg2.L*sin(3*obj.alpha1-obj.Q(i,3));
			    x(i,5) = x0(i)+obj.Leg1.L*sin(4*obj.alpha1-obj.Q(i,3));
			    x(i,6) = x0(i)-obj.Leg2.L*sin(3*obj.alpha1+obj.Q(i,3));
			    x(i,7) = x0(i)-obj.Leg1.L*sin(2*obj.alpha1+obj.Q(i,3));
			    x(i,8) = x0(i)-obj.Leg2.L*sin(obj.alpha1+obj.Q(i,3));

			    y0(i) = obj.Q(i,2)+obj.Leg1.L*cos(obj.Q(i,3));
			    y(i,1) = obj.Q(i,2);
			    y(i,2) = y0(i)-obj.Leg2.L*cos(obj.alpha1-obj.Q(i,3));
			    y(i,3) = y0(i)-obj.Leg1.L*cos(2*obj.alpha1-obj.Q(i,3));
			    y(i,4) = y0(i)-obj.Leg2.L*cos(3*obj.alpha1-obj.Q(i,3));
			    y(i,5) = y0(i)-obj.Leg1.L*cos(4*obj.alpha1-obj.Q(i,3));
			    y(i,6) = y0(i)-obj.Leg2.L*cos(3*obj.alpha1+obj.Q(i,3));
			    y(i,7) = y0(i)-obj.Leg1.L*cos(2*obj.alpha1+obj.Q(i,3));
			    y(i,8) = y0(i)-obj.Leg2.L*cos(obj.alpha1+obj.Q(i,3));

			    % Get critical point of foot
			    angle1 = obj.Q(i,3)+obj.alpha1/2-obj.phi; % Angle of foot

			    x_c(i,1) = x(i,1)+0.25*sin(angle1);
			    x_c(i,2) = x0(i)+l*sin(2*obj.alpha1-obj.Q(i,3) + 0.1234);
			    x_c(i,3) = x0(i)+l*sin(4*obj.alpha1-obj.Q(i,3) + 0.1234);
			    x_c(i,4) = x0(i)+l*sin(6*obj.alpha1+obj.Q(i,3) + 0.1234);

			    y_c(i,1) = y(i,1)+0.25*cos(angle1);
			    y_c(i,2) = y0(i)-l*cos(2*obj.alpha1-obj.Q(i,3) + 0.1234);
			    y_c(i,3) = y0(i)-l*cos(4*obj.alpha1-obj.Q(i,3) + 0.1234);
			    y_c(i,4) = y0(i)-l*cos(6*obj.alpha1-obj.Q(i,3) + 0.1234);

			    t1 = linspace(3*pi/2-angle1, 11*pi/6-angle1)';
			    x_f(:,1,i) = x_c(i,1) + 0.25*cos(t1);
			    y_f(:,1,i) = y_c(i,1) + 0.25*sin(t1);
				
				x_f(:,2,i) = x_c(i,2) + 0.25*cos(t1+2*obj.alpha1);
				y_f(:,2,i) = y_c(i,2) + 0.25*sin(t1+2*obj.alpha1);

				x_f(:,3,i) = x_c(i,3) + 0.25*cos(t1+4*obj.alpha1);
				y_f(:,3,i) = y_c(i,3) + 0.25*sin(t1+4*obj.alpha1);

				x_f(:,4,i) = x_c(i,4) + 0.25*cos(t1+6*obj.alpha1);
				y_f(:,4,i) = y_c(i,4) + 0.25*sin(t1+6*obj.alpha1);

			    % if obj.Leg1.form == 'arc':
			    % 	t = linspace()
			end

			

			figure(100)
			slope_line = plot([-(x0(1)+5)*cos(obj.phi) (x0(end)+5)*cos(obj.phi)],[(x0(1)+5)*sin(obj.phi) -(x0(end)+5)*sin(obj.phi)],'k','linewidth',2);hold on; % Draw the slope
			stance_leg = plot([x(1,1) x0(1)],[y(1,1) y0(1)],'r','linewidth',2.5);hold on; % Draw stance leg
			leg_26 = plot([x(1,2) x(1,6)],[y(1,2) y(1,6)],'b','linewidth',2);hold on; % Draw 2 and 6 legs
			leg_37 = plot([x(1,3) x(1,7)],[y(1,3) y(1,7)],'b','linewidth',2);hold on; % Draw 3 and 7 legs
			leg_48 = plot([x(1,4) x(1,8)],[y(1,4) y(1,8)],'b','linewidth',2);hold on; % Draw 4 and 8 legs
			leg_5 = plot([x(1,5) x0(1)],[y(1,5) y0(1)],'b','linewidth',2);hold on;  % Draw 5 leg

			% Plot four feet
			% foot_coc = plot(x_c(1,:),y_c(1,:),'ro');	
			foot1 = fill(x_f(:,1,1), y_f(:,1,1),'k');
			foot2 = fill(x_f(:,2,1), y_f(:,2,1),'k');
			foot3 = fill(x_f(:,3,1), y_f(:,3,1),'k');
			foot4 = fill(x_f(:,4,1), y_f(:,4,1),'k');
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
			    set(foot1,'Vertices', [x_f(:,1,i), y_f(:,1,i)]);
			    set(foot2,'Vertices', [x_f(:,2,i), y_f(:,2,i)]);
			    set(foot3,'Vertices', [x_f(:,3,i), y_f(:,3,i)]);
			    set(foot4,'Vertices', [x_f(:,4,i), y_f(:,4,i)]);

			    % set(foot_coc,'Xdata',x_c(i,:),'Ydata',y_c(i,:));

			    title(sprintf('Time = %f[sec]',obj.T(i)),'Color','w');
			    drawnow;
			end
		end
	end
end