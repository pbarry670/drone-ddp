function [c, c_terminal] = cost_file(Q, R, Qf, target_state)

% Function that returns function handles that compute the cost and terminal
% cost.

% cost is a function of (x, u)
% terminal cost is a function of (x)

    function [c, cx, cu, cxx, cxu, cuu] = costfn(x, u)
        err = x - target_state;
        c = 0.5 * (err' * Q * err + u' * R * u);
        cx = Q * err;
        cxx = Q;
        cxu = 0;
        cu = R * u;
        cuu = R;
    end

    function [c, cx, cxx] = term_costfn(x)
        err = x - target_state;
        c = 0.5 * (err' * Qf * err);
        cx = Qf * err;
        cxx = Qf;
    end

c = @costfn;
c_terminal = @term_costfn;
end