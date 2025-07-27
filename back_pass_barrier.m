function [controller] = back_pass_barrier(states, controls, dyn, costfn, term_costfn, regularizer)
%BACK_PASS The backward pass of iLQR / DDP.
%
%   states: A tf-by-n matrix where the row t contains the state of the
%   system at time step t. These states are to be used for Taylor
%   approximations.
%
%   controls: A tf-by-m matrix where the row t contains the control applied
%   at time step t. These controls are to be used for Taylor
%   approximations.
%
%   dyn: A function defining the robot dynamics and pertinent derivatives.
%   It has the form:
%
%       [next_state, fx, fu, fxx, fxu, fuu] = dynamics_wrapper(state, control)
%
%   Note that, if the dynamics are f(x, u), the second-order derivatives of
%   the i-th entry of f(x, u) are fxx(i, :, :), fxu(i, :, :), fuu(i, :, :).
%
%   regularizer: The value to use for the regularization parameter
%   (defaults to 0).
%
%
%   All derivatives are evaluated at (state, control).
%
%   costfn: A function that computes the cost at each time step before the
%   terminal step. It also evaluates the pertinent derivatives at the
%   current state and control. It has the form:
%
%       [cost, cx, cu, cxx, cxu, cuu] = costfn(state, control).
%
%   term_costfn: A function that computes the terminal cost as well as the
%   derivatives of the terminal cost function at a given state. It has the
%   form:
%
%       [cost, cx, cxx] = term_costfn(state)
%
%   Returns:
%
%   controller: A struct with 4 entries defining the controller
%
%       controller.K: A (tf - 1)-by-m-by-n matrix defining the linear
%       feedback portion of the controller.
%
%       controller.k: A (tf - 1)-by-m matrix defining the affine term of
%       the controller.
%
%       controller.states: A tf-by-n matrix containing the states used
%       during the backward pass for Taylor expansions (i.e., x_bar).
%
%       controller.controls: A (tf - 1)-by-m matrix containing the controls
%       used during the backward pass for Taylor expansions (i.e., u_bar). 
%

% Some basic setup code
horizon = size(controls, 1);
n = size(states, 2);
m = size(controls, 2);
dynamic_regularizer = 1; %this is the changing regularizer mu (starts at 1, can change)
regularizer_scaling = 1.1; %value to scale mu by during regularization iteration
min_eigval = 1e-6; %minimum eigenvalue for dynamic regularizer to satisfy
cond_threshold = 1e3;

controller = struct;
controller.K = zeros(horizon, m, n);
controller.k = zeros(horizon, m);
controller.states = states;
controller.controls = controls;

%% Fill in your code below
w_tf = barrier_file(controller.states(end, :)');
[~, Vtf_x, Vtf_xx] = term_costfn(controller.states(end, :)', w_tf);

Vt_next_x = Vtf_x;
Vt_next_xx = Vtf_xx;

for t = horizon:-1:1


    [next_state, fx, fu, fxx, fxu, fuu] = dyn(controller.states(t,:)', controller.controls(t,:));

    w = barrier_file(next_state);
    [cost, cx, cu, cxx, cxu, cuu] = costfn(controller.states(t, :)', controller.controls(t,:)', w);
    
    Qt_x = cx + fx'*Vt_next_x;
    Qt_u = cu + fu'*Vt_next_x;

    Qt_xx = cxx + fx'*Vt_next_xx*fx + tensor_product(Vt_next_x, fxx) + regularizer*(fx'*fx);
    Qt_xu = cxu + fx'*Vt_next_xx*fu + tensor_product(Vt_next_x, fxu) + regularizer*(fx'*fu);

    Qt_uu = cuu + fu'*Vt_next_xx*fu + tensor_product(Vt_next_x, fuu) + regularizer*(fu'*fu);
    dynamic_regularizer = 1;
    while min(abs(eig(Qt_uu))) < min_eigval || cond(Qt_uu) > cond_threshold
        dynamic_regularizer = regularizer_scaling*dynamic_regularizer;
        Qt_uu = Qt_uu + dynamic_regularizer * eye(size(cuu));
    end

    Qt_ux = Qt_xu';

    inv_Qt_uu = pinv(Qt_uu);

    controller.K(t, :, :) = -inv_Qt_uu*Qt_ux;
    controller.k(t, :) = -inv_Qt_uu*Qt_u;

    Vt_next_x = Qt_x - Qt_xu*inv_Qt_uu*Qt_u;
    Vt_next_xx = Qt_xx - Qt_xu*inv_Qt_uu*Qt_ux;
end

end

function[val]= tensor_product(V, f)

n = length(V);
result = 0;

for i = 1:n

    component = V(i)*squeeze(f(i, :, :));
    result = result + component;

end

val = result;

end