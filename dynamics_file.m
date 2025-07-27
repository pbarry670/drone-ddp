function [dyn] = dynamics_file(m, I, t_constant, l_arm, gravity, dt)
% Function for all dynamics

x = sym('x');
y = sym('y');
z = sym('z');
xdot = sym('xdot');
ydot = sym('ydot');
zdot = sym('zdot');
phi = sym('phi');
theta = sym('theta');
psi = sym('psi');
p = sym('p');
q = sym('q');
r = sym('r');
u1 = sym('u1');
u2 = sym('u2');
u3 = sym('u3');
u4 = sym('u4');

R = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
     sin(theta)*sin(phi)*cos(psi) - cos(phi)*sin(psi), sin(theta)*sin(phi)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta);
     sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi), sin(theta)*sin(psi)*cos(phi) - sin(phi)*sin(psi), cos(theta)*cos(phi)];

pos_dot = [xdot; ydot; zdot]; % d/dt([x;y;z])
vel_dot = [(1/m)*(R*[0;0;u1 + u2 + u3 + u4]+[0;0;-m*gravity])]; %[xdotdot; ydotdot, zdotdot]
eulang_dot = [1, tan(theta)*sin(phi), tan(theta)*cos(phi); 0, cos(phi), -sin(phi); 0, sin(phi)/cos(theta), cos(phi)/cos(theta)]*[p;q;r];

p_dot = (1/I(1,1))*((l_arm*sqrt(2)/2)*(u1+u3-u2-u4) - (I(3,3)-I(2,2))*q*r);
q_dot = (1/I(2,2))*((l_arm*sqrt(2)/2)*(u3+u4-u1-u2) + (I(3,3)-I(1,1))*p*r);
r_dot = (t_constant/I(3,3))*(u1+u4-u2-u3);
angvel_dot = [p_dot; q_dot; r_dot];

state_dot = [pos_dot; vel_dot; eulang_dot; angvel_dot];
state_vec = [x; y; z; xdot; ydot; zdot; phi; theta; psi; p; q; r];
control_vec = [u1; u2; u3; u4];
vars = [state_vec; control_vec];

F = [state_vec + dt*state_dot]; % This can be a better integration scheme but is fine for now

fx = jacobian(F, state_vec);
fu = jacobian(F, control_vec);

numeric_f = matlabFunction(F, Vars=vars);
numeric_fx = matlabFunction(fx, Vars=vars);
numeric_fu = matlabFunction(fu, Vars=vars);

for i = 1:length(state_vec)
    numeric_fxx{i} = matlabFunction(hessian(F(i), state_vec), Vars=vars);
    numeric_fxu{i} = matlabFunction(jacobian(jacobian(F(i), state_vec), control_vec), Vars=vars);
    numeric_fuu{i} = matlabFunction(hessian(F(i), control_vec), Vars=vars);
end

dyn = @(x, u) dynamics_wrapper(x, u, numeric_f, numeric_fx, numeric_fu, numeric_fxx, numeric_fxu, numeric_fuu);

end


function [wrapper_f, wrapper_fx, wrapper_fu, wrapper_fxx, wrapper_fxu, wrapper_fuu]= dynamics_wrapper(state, control, num_f, num_fx, num_fu, num_fxx, num_fxu, num_fuu)

    x_val = state(1);
    y_val = state(2);
    z_val = state(3);
    xdot_val = state(4);
    ydot_val = state(5);
    zdot_val = state(6);
    phi_val = state(7);
    theta_val = state(8);
    psi_val = state(9);
    p_val = state(10);
    q_val = state(11);
    r_val = state(12);
    
    u1_val = control(1);
    u2_val = control(2);
    u3_val = control(3);
    u4_val = control(4);
    
    wrapper_f = num_f(x_val, y_val, z_val, xdot_val, ydot_val, zdot_val, phi_val, theta_val, psi_val, p_val, q_val, r_val, u1_val, u2_val, u3_val, u4_val);
    wrapper_fx = num_fx(x_val, y_val, z_val, xdot_val, ydot_val, zdot_val, phi_val, theta_val, psi_val, p_val, q_val, r_val, u1_val, u2_val, u3_val, u4_val);
    wrapper_fu = num_fu(x_val, y_val, z_val, xdot_val, ydot_val, zdot_val, phi_val, theta_val, psi_val, p_val, q_val, r_val, u1_val, u2_val, u3_val, u4_val);
    
    num_states = length(state);
    num_controls = length(control);
    
    wrapper_fxx = zeros(num_states, num_states, num_states);
    wrapper_fxu = zeros(num_states, num_states, num_controls);
    wrapper_fuu = zeros(num_states, num_controls, num_controls);
    
    for i = 1:num_states
        wrapper_fxx(i, :, :) = num_fxx{i}(x_val, y_val, z_val, xdot_val, ydot_val, zdot_val, phi_val, theta_val, psi_val, p_val, q_val, r_val, u1_val, u2_val, u3_val, u4_val);
        wrapper_fxu(i, :, :) = num_fxu{i}(x_val, y_val, z_val, xdot_val, ydot_val, zdot_val, phi_val, theta_val, psi_val, p_val, q_val, r_val, u1_val, u2_val, u3_val, u4_val);
        wrapper_fuu(i, :, :) = num_fuu{i}(x_val, y_val, z_val, xdot_val, ydot_val, zdot_val, phi_val, theta_val, psi_val, p_val, q_val, r_val, u1_val, u2_val, u3_val, u4_val);
    end

end