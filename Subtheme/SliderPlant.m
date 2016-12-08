classdef SliderPlant
	% Slider Class, method and properties of it

	properties (SetAccess = private)
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
		q = [0 0 pi/6 0 0 0 0 0]'; % x,z,th1,th2
		simulation_time = 5;
	end

	methods
		function obj=SliderPlant(q, simulation_time)
			if (nargin>0)
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

			h = zeros(3,1);
			h(1,1) = -obj.a*obj.q(7)^2*(obj.m1+obj.m2)*sin(obj.q(3)) + obj.b*obj.q(8)^2*obj.m2*sin(obj.q(4));
			h(2,1) = (obj.m1+obj.m2)*(obj.g-obj.a*obj.q(7)^2*cos(obj.q(3))) + obj.b*obj.q(8)^2*obj.m2*cos(obj.q(4));
			h(3,1) = -obj.a*obj.g*(obj.m1+obj.m2)*sin(obj.q(3))+obj.b*obj.q(7)^2*obj.m2*sin(obj.q(7)-obj.q(8));
			h(4,1) = obj.b*obj.m2*(obj.a*obj.q(7)^2*sin(obj.q(7)-obj.q(8)) + obj.g*sin(obj.q(8)));

		end

		function N = get.N(obj)

		    J = [ 1 0 obj.R*(cos(obj.q(3))-1) 0;
		          0 1 -obj.R*sin(obj.q(3)) 0];
		    J_dot = [0 0 -obj.R*sin(obj.q(3))*obj.q(7) 0;
		    		0 0 -obj.R*cos(obj.q(3))*obj.q(7) 0];
		    lambda = inv(J*inv(obj.M)*J')*(J*inv(obj.M)*obj.h-J_dot*obj.q(5:8));
		    N = J'*lambda;
		end

		function dq = slider_dynamic(obj,t,q)
			obj.q = q;
			dq = zeros(8,1);
			dq(1:4,1) = q(5:8);
			dq(5:8,1) = inv(obj.M)*(obj.N-obj.h);
		end
	end

end
