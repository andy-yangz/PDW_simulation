classdef SliderPlant
	% Slider Class, method and properties of it
	properties (GetAccess=private)
		m1 = 1.0; 
		m2 = 1.0;
		a = 0.3;
		b = 0.2;
		R = 1.0243;
		I1 = 0.1; 
		I2 = 0.1;
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
		q = [0 0 0 pi/6 0 0 0 0]'; % x,z,th1,th2
		simulation_time = 3;
	end

	methods
		function obj=SliderPlant(q0, simulation_time)
			if (nargin>0):
				obj.q0 = q0;
				obj.simulation_time = simulation_time;
			end
		end

		function M = get.M(obj,q)
			M = zeros(4,4);
			M(1,1) = obj.m1+obj.m2;
			M(1,2) = 0;
			M(1,3) = obj.a*(obj.m1+obj.m2)*cos(q(3));
			M(1,4) = -b*m2*cos(q(4));
			M(2,1) = 0;
			M(2,2) = obj.m1+obj.m2;
			M(2,3) = -obj.a*(obj.m1+obj.m2)*sin(q(3));
			M(2,4) = b*m2*sin(q(4));
			M(3,1) = obj.a*(obj.m1+obj.m2)*cos(q(3));
			M(3,2) = -obj.a*(obj.m1+obj.m2)*sin(q(3));
			M(3,3) = I1+obj.a^2*(obj.m1+obj.m2);
			M(3,4) = -obj.a*b*m2*cos(q(3)-q(4));
			M(4,1) = -b*m2*cos(q(4));
			M(4,2) = b*m2*sin(q(4));
			M(4,3) = -obj.a*b*m2*cos(q(3)-q(4));
			M(4,4) = I2+b^2*m2;
		end

		function h = get.h(obj,q)

			h = zeros(3,1);
			h(1,1) = -obj.a*q(7)^2*(m1+m2)*sin(q(3)) + b*q(8)^2*m2*sin(q(4));
			h(2,1) = (m1+m2)*(g-obj.a*q(7)^2*cos(q(3))) + b*q(8)^2*m2*cos(q(4));
			h(3,1) = -obj.a*g*(m1+m2)*sin(q(3))+b*q(7)^2*m2*sin(q(7)-q(8));
			h(4,1) = b*m2*(obj.a*q(7)^2*sin(q(7)-q(8)) + g*sin(q(8)));

		end

		function N = get_N(obj,q)

		    parameter;

		    M = get_M(q);
		    h = get_h(q);


		    J = [ 1 0 R*(cos(q(3))-1) 0;
		          0 1 -R*sin(q(3)) 0];
		    J_dot = [0 0 -R*sin(q(3))*q(7) 0;
		    		0 0 -R*cos(q(3))*q(7) 0];
		    lambda = inv(J*inv(M)*J')*(J*inv(M)*h-J_dot*q(5:8));
		    N = J'*lambda;
		end
