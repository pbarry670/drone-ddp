function [w]= barrier_file(state)

target_state = [5; 3; 2; 0; 0; 0; 0; 0; 0; 0; 0; 0];

h1 = @(x, y, z) ((x-2.2)^2) + ((y - 2.2)^2) + ((z-1)^2) - 1;
%h1 = @(x, y, z) ((x-1.82)^2) + ((y - 1.05)^2) + ((z-2.62)^2) - 1;

h2 = @(x, y, z) x^2 + ((y + 0.2)^2) + z^2 - 1;

h3 = @(x, y, z) ((x-3)^2) + y^2 + ((z-0.5)^2) - 1;


x_i = state(1);
y_i = state(2);
z_i = state(3);

x_goal = target_state(1);
y_goal = target_state(2);
z_goal = target_state(3);

w1 = 1/h1(x_i, y_i, z_i) - 1/h1(x_goal, y_goal, z_goal);
w2 = 1/h2(x_i, y_i, z_i) - 1/h2(x_goal, y_goal, z_goal);
w3 = 1/h3(x_i, y_i, z_i) - 1/h3(x_goal, y_goal, z_goal);

w = [w1; w2; w3];


end

