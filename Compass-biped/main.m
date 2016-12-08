
cleal; close all; clc;
parameter; % Introduce paremeters

time = []; % Variables include time and simulation result
result = [];

tspan = [0 5];
options = odeset('Event', @collision, 'RelTol', 1e-12, 'AbsTol',1e-12*ones(1,8));


for step_number = 1:10
	if tspan (1)