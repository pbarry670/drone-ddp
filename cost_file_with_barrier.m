function [c, c_terminal] = cost_file_with_barrier(Q, R, Qf, a_barrier, target_state)

% Function that returns function handles that compute the cost and terminal
% cost.

% cost is a function of (x, u, w)
% terminal cost is a function of (x, w)

    function [c, cx, cu, cxx, cxu, cuu] = costfn(x, u, w_barrier)
        err = x - target_state;
        c = 0.5 * (err' * Q * err + u' * R * u) + a_barrier*(w_barrier'*w_barrier);
        cx = Q * err;
        cxx = Q;
        cxu = 0;
        cu = R * u;
        cuu = R;
    end

    function [c, cx, cxx] = term_costfn(x, w_barrier)
        err = x - target_state;
        c = 0.5 * (err' * Qf * err) + a_barrier*(w_barrier'*w_barrier);
        cx = Qf * err;
        cxx = Qf;
    end

c = @costfn;
c_terminal = @term_costfn;
end