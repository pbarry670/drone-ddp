function [dyn] = planar_quadrotor(mass, inertia, radius, gravity, dt)
%PLANAR_QUADROTOR Summary of this function goes here
%   Detailed explanation goes here

x = sym('x');
y = sym('y');
theta = sym('theta');
xdot = sym('xdot');
ydot = sym('ydot');
thetadot = sym('thetadot');
u1 = sym('u1');
u2 = sym('u2');

coord = [x; y; theta];
vel = [xdot; ydot; thetadot];
state = [coord; vel];
controls = [u1; u2];
vars = [state; controls];

eom = [-(u1 + u2) * sin(theta) / mass;
        (u1 + u2) * cos(theta) - mass * gravity;
        (radius / inertia) * (u1 - u2)];

f = [coord + dt * vel;
     vel + dt * eom];

fx = jacobian(f, state);
fu = jacobian(f, controls);

numeric_f = matlabFunction(f, Vars=vars);
numeric_fx = matlabFunction(fx, Vars=vars);
numeric_fu = matlabFunction(fu, Vars=vars);

numeric_fxx = {};
numeric_fxu = {};
numeric_fuu = {};

for i = 1:6
    numeric_fxx{i} = matlabFunction(hessian(f(i), state), Vars=vars);
    numeric_fxu{i} = matlabFunction(jacobian(jacobian(f(i), state), controls), Vars=vars);
    numeric_fuu{i} = matlabFunction(hessian(f(i), controls), Vars=vars);
end

dyn = @(x, u) dynamics_wrapper(x, u, numeric_f, numeric_fx, numeric_fu, numeric_fxx, numeric_fxu, numeric_fuu);

end


function [wrapper_f, wrapper_fx, wrapper_fu, wrapper_fxx, wrapper_fxu, wrapper_fuu] = dynamics_wrapper(state, control, num_f, num_fx, num_fu, num_fxx, num_fxu, num_fuu)
    x_val = state(1);
    y_val = state(2);
    theta_val = state(3);
    xdot_val = state(4);
    ydot_val = state(5);
    zdot_val = state(6);

    u1_val = control(1);
    u2_val = control(2);

    wrapper_f = num_f(x_val, y_val, theta_val, xdot_val, ydot_val, zdot_val, u1_val, u2_val);
    wrapper_fx = num_fx(x_val, y_val, theta_val, xdot_val, ydot_val, zdot_val, u1_val, u2_val);
    wrapper_fu = num_fu(x_val, y_val, theta_val, xdot_val, ydot_val, zdot_val, u1_val, u2_val);

    wrapper_fxx = zeros(6, 6, 6);
    wrapper_fxu = zeros(6, 6, 2);
    wrapper_fuu = zeros(6, 2, 2);

    for i = 1:6
        wrapper_fxx(i, :, :) = num_fxx{i}(x_val, y_val, theta_val, xdot_val, ydot_val, zdot_val, u1_val, u2_val);
        wrapper_fxu(i, :, :) = num_fxu{i}(x_val, y_val, theta_val, xdot_val, ydot_val, zdot_val, u1_val, u2_val);
        wrapper_fuu(i, :, :) = num_fuu{i}(x_val, y_val, theta_val, xdot_val, ydot_val, zdot_val, u1_val, u2_val);
    end
end

