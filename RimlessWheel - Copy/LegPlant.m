classdef LegPlant<handle
	% LegPlant is the class define a leg

	properties 
		L = 1; % Leg length
		form = 'point'; % Leg form variables, you can define it as point or arc
		R = 0; % The readius of foot, if it's point foot, R = 0;
	end
	properties (Dependent)
		J; % Jacobian matric for constraint
	end

	methods
		function obj = LegPlant(form, R)
			if (nargin>0)
				obj.form = form;
				obj.R = R;
			end
		end
	end
end