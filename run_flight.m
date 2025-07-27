close all;
clear;
clc;
global Q
global Qf
global R
global target_state
%% Initialize system quantities and initial conditions
mass = 0.5; %kg
I = [0.0032, 0, 0;
     0, 0.0032, 0
     0, 0, 0.0055]; %kgm^2
t_constant = 0.01691; %1/m
l_arm = 0.17; %m
gravity = 9.81; %m/s^2

dt = 0.01;

tf = 800; % tf*dt should equal 8 seconds

ic = [-3; -2; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0]; %x, y, z, xdot, ydot, zdot, phi, theta, psi, p, q, r
target_state = [5; 3; 2; 0; 0; 0; 0; 0; 0; 0; 0; 0];

dyn = dynamics_file(mass, I, t_constant, l_arm, gravity, dt);

Q = eye(12); %CHANGE

% Tune Q
    Q(1,1) = 1; % cost on x
     Q(2,2) = 1.15; % cost on y
    Q(3,3) = 14; % cost on z
    Q(4,4) = 0.1; % cost on xdot
    Q(5,5) = 0.1; % cost on ydot
    Q(6,6) = 7; % cost on zdot
    Q(7,7) = 0; % cost on phi
    Q(8,8) = 0; % cost on theta
    Q(9,9) = 0; % cost on psi
    Q(10,10) = 0; % cost on p
    Q(11,11) = 0; % cost on q
    Q(12,12) = 0; % cost on r

    R = 10*eye(4); %

    Qf = 140*Q;

    regularizer = 1;

%% Initialize cost and run DDP
[costfn, term_costfn] = cost_file(Q, R, Qf, target_state);

iters = 5;

initial_controls = (mass * gravity / 4) * ones(tf,4); %hover

[controller, total_costs] = ddp(ic, initial_controls, iters, regularizer, dyn, costfn, term_costfn);

%% Use DDP-optimized controller with forward pass to visualize trajectory
[states, controls, costs] = fwd_pass(ic, controller, dyn, costfn, term_costfn);

final_state = states(end, 1:3);
target_values = [5, 3, 2];
percent_error = 100*(final_state-target_values)./target_values;
fprintf('Final State: x=%.4f, y=%.4f, z=%.4f\n', final_state);
fprintf('Percent Error: x=%.2f%%, y=%.2f%%, z=%.2f%%\n', abs(percent_error));

%visualization assumes vertical vectors
x_states = states(:,1);
y_states = states(:,2);
z_states = states(:,3);
%testing trajectory visualization with direct trajectory
% x_states = linspace(-3, 5, 100)'
% y_states = linspace(-2, 3, 100)'
% z_states = linspace(-1, 2, 100)'
%rectangular prism "quadcopter"
length = 0.5;
width = 0.5;
height = 0.1;
figure;
hold on;
grid on;
plot3(-3, -2, -1, 'k.', 'MarkerSize', 15); %start pos
plot3(5, 3, 2, 'kx', 'MarkerSize', 25, 'LineWidth', 2); %desired endpos
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
view(3); %set 3d view
%making the prism nodes ref from center of prism
vertices = [
    -length/2, -width/2, -height/2;
     length/2, -width/2, -height/2;
     length/2,  width/2, -height/2;
    -length/2,  width/2, -height/2;
    -length/2, -width/2,  height/2;
     length/2, -width/2,  height/2;
     length/2,  width/2,  height/2;
    -length/2,  width/2,  height/2;
];
%connecting nodes to make faces
faces = [
    1, 2, 3, 4; %bottom
    5, 6, 7, 8; %top
    1, 2, 6, 5; %side face 1
    2, 3, 7, 6; %side face 2
    3, 4, 8, 7; %side face 3
    4, 1, 5, 8; %side face 4
];
%making the prism
prism = patch('Vertices', vertices, 'Faces', faces, ...
    'FaceColor', 'black', 'EdgeColor', 'black', 'FaceAlpha', 0.8);
%initialize traj line
trajectory = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 1.5);
%animate trajectory
for i = 1:size(x_states,1)
    %updating prism position
    updatedVertices = vertices + [x_states(i), y_states(i), z_states(i)];
    set(prism, 'Vertices', updatedVertices);
    %update trajectory and add curr pos to trajectory line
    trajectory.XData = [trajectory.XData, x_states(i)];
    trajectory.YData = [trajectory.YData, y_states(i)];
    trajectory.ZData = [trajectory.ZData, z_states(i)];
    drawnow limitrate;
    pause(0); %controls animation speed, not a function of dynamics time
end


%% States Visualization
figure('Name', 'Quadrotor States', 'NumberTitle', 'off');

t = linspace(0, 8, size(states, 1)); %time

%positions
subplot(2, 2, 1);
hold on;
plot(t, states(:, 1), 'LineWidth', 1.2);
plot(t, states(:, 2), 'LineWidth', 1.2);
plot(t, states(:, 3), 'LineWidth', 1.2);
xlabel('Time (s)'); 
ylabel('Position'); 
legend('x', 'y', 'z', 'location', 'southeast');
title('Positions'); 
grid on;
box on;

%velocities
subplot(2, 2, 2);
hold on;
plot(t, states(:, 4), 'LineWidth', 1.2);
plot(t, states(:, 5), 'LineWidth', 1.2);
plot(t, states(:, 6), 'LineWidth', 1.2);
xlabel('Time (s)'); 
ylabel('Velocity'); 
legend('xdot', 'ydot', 'zdot', 'location', 'northeast');
title('Velocities'); 
grid on;
box on;

%euler angles
subplot(2, 2, 3);
hold on;
plot(t, states(:, 7), 'LineWidth', 1.2);
plot(t, states(:, 8), 'LineWidth', 1.2);
plot(t, states(:, 9), 'LineWidth', 1.2);
xlabel('Time (s)'); 
ylabel('Euler Angles (rad)'); 
legend('\phi', '\theta', '\psi', 'location', 'northeast');
title('Euler Angles'); 
grid on;
box on;

%angular rates
subplot(2, 2, 4);
hold on;
plot(t, states(:, 10), 'LineWidth', 1.2);
plot(t, states(:, 11), 'LineWidth', 1.2);
plot(t, states(:, 12), 'LineWidth', 1.2);
xlabel('Time (s)'); 
ylabel('Angular Rates (rad/s)'); 
legend('p', 'q', 'r', 'location', 'southeast');
title('Angular Rates'); 
grid on;
box on;

%% Controls Visualization
t = linspace(0, 8, size(controls, 1)); % Assuming 8 seconds and 800 timesteps
figure('Name', 'Quadrotor Controls', 'NumberTitle', 'off');
hold on;
plot(t, controls(:, 1), 'LineWidth', 0.8)
plot(t, controls(:, 2), 'LineWidth', 0.8)
plot(t, controls(:, 3), 'LineWidth', 0.8)
plot(t, controls(:, 4), 'LineWidth', 0.8)
xlabel('Time (s)');
ylabel('Control Inputs');
legend('u1', 'u2', 'u3', 'u4');
title('Control Inputs Over Time');
grid on;
box on;