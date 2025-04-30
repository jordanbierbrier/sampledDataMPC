function data = setupReachability(B_d, u_min, u_max)
% Prepares static data for reachability computation.
% This should be called once, then reused.

    % Input validation
    [n, m] = size(B_d);
    assert(all(size(u_min) == [m, 1]) && all(size(u_max) == [m, 1]), ...
        'u_min and u_max must be m x 1');

    % Define input box as a Polyhedron (V-rep and H-rep both available)
    U = Polyhedron('lb', u_min, 'ub', u_max);  % input constraint set

    % Store in struct
    data.B_d = B_d;
    data.U = U;
end