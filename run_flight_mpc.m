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
tf = 800; % 8 seconds, or 1000 time steps
sim_horizon = tf;

ic = [-3; -2; -1; 0; 0; 0; 0; 0; 0; 0; 0; 0]; %x, y, z, xdot, ydot, zdot, phi, theta, psi, p, q, r
target_state = [5; 3; 2; 0; 0; 0; 0; 0; 0; 0; 0; 0];

dyn = dynamics_file(mass, I, t_constant, l_arm, gravity, dt);

%% Tuning
Q = eye(12); %CHANGE

% Tune Q
    Q(1,1) = 6; % cost on x
    Q(2,2) = 6; % cost on y
    Q(3,3) = 30; % cost on z
    Q(4,4) = 0.5; % cost on xdot
    Q(5,5) = 0.5; % cost on ydot
    Q(6,6) = 0.2; % cost on zdot
    Q(7,7) = 0; % cost on phi
    Q(8,8) = 0; % cost on theta
    Q(9,9) = 0; % cost on psi
    Q(10,10) = 0; % cost on p
    Q(11,11) = 0; % cost on q
    Q(12,12) = 0; % cost on r
    
R = 7*eye(4); %CHANGE

Qf = 350*Q;

regularizer = 1;

ddp_iters = 2;
planning_horizon = 20;
initial_controls = ((mass*gravity)/4)*ones(planning_horizon - 1, 4);

%% Initialize cost and run DDP
[costfn, term_costfn] = cost_file(Q, R, Qf, target_state);

[states, controls] = mpc_func(ic, sim_horizon, initial_controls, ddp_iters, regularizer, dyn, costfn, term_costfn);
%% Trajectory Visualization

figure;
hold on;
grid on;
plot3(-3, -2, -1, 'k.', 'MarkerSize', 15); %start pos
plot3(5, 3, 2, 'kx', 'MarkerSize', 25, 'LineWidth', 2); %desired endpos
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal;
view(3); %set 3d view

actual_states = squeeze(states(:, end, :));

init_planned = squeeze(states(1, :, 1:3));
actual_traj = squeeze(states(:, end, 1:3));
actual_traj = [-3 -2 -1; actual_traj];
plot3(init_planned(:, 1), init_planned(:, 2), init_planned(:, 3), '--r', 'LineWidth', 1.5)
hold on
for i = 1:sim_horizon/planning_horizon

    time_step = i*planning_horizon;
    hold on
    current_planned = squeeze(states(time_step, :, 1:3));
    plot3(current_planned(:,1), current_planned(:,2), current_planned(:,3), '--r', 'LineWidth', 1.5)
    pause(0.2)
    hold on
    current_actual = actual_traj(1:time_step, :);
    plot3(current_actual(:,1), current_actual(:,2), current_actual(:,3), 'b', 'LineWidth', 1.5)
    pause(0.2)

end

final_state = actual_traj(end, 1:3);
target_values = [5, 3, 2];
percent_error = 100*(final_state-target_values)./target_values;
fprintf('Final State: x=%.4f, y=%.4f, z=%.4f\n', final_state);
fprintf('Percent Error: x=%.2f%%, y=%.2f%%, z=%.2f%%\n', abs(percent_error));


%% States Visualization
figure('Name', 'Quadrotor States', 'NumberTitle', 'off');

states = actual_states;
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
