function [H, h] = reachableSet(A_d, x, data)
% Computes one-step reachable set R(x, T_s) as H*y <= h
% using precomputed data from setup_reachability()

    % Extract
    B_d = data.B_d;
    U = data.U;

    % Apply affine map: y = A_d * x + B_d * u
    Y = A_d * x + B_d * U;

    % Convert to minimal H-representation
    Y.minHRep();
    H = Y.A;
    h = Y.b;
end