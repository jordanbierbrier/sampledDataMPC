function [val, u_opt] = findMargin(A_d, B_d, x, Qb, lb, Mb, bb, c, u_min, u_max)
    % Dimensions
    n = size(A_d, 1);
    m = size(B_d, 2);
    
    % % y = A_d * x + B_d * u
    % Ax = A_d * x;
    % B = B_d;
    % 
    % % h2 terms (constant in u): keep full form for clarity
    % h2_const = x' * Qb * x + lb' * x + c;
    % h2_linear = (Mb * x + bb)';   % multiplies u later
    % 
    % % h3 terms
    % % Expand: (Ax + Bu)' * Qb * (Ax + Bu)
    % H3_quad = B' * Qb * B;                         % u' H3_quad u
    % H3_linear = 2 * Ax' * Qb * B + lb' * B + ...   % (linear in u)
    %             (Mb * Ax)' + bb';                 % from (Mb*y + bb)' * u
    % 
    % % Final cost: h2 - h3
    % H = 0.5 * (H3_quad);              % Only from h3 (since h2 is linear in u)
    % f = (H3_linear - h2_linear);      % sign flip since it's h2 - h3
    % 
    % % Call quadprog
    % options = optimoptions('quadprog', 'Display', 'off');
    % [u_opt, val] = quadprog(2*H, f', [], [], [], [], u_min, u_max, [], options);
    % 
    % % Final value = h2 - h3
    % y = A_d * x + B_d * u_opt;
    % h2 = x' * Qb * x + lb' * x + c + (Mb * x + bb)' * u_opt;
    % h3 = y' * Qb * y + lb' * y + c + (Mb * y + bb)' * u_opt;
    % nu_val = h2 - h3;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % y = A_d * x + B_d * u
    Ax = A_d * x;
    B = B_d;

    % h2 terms (constant in u): keep full form for clarity
    h2_const = x' * Qb * x + lb' * x + c;
    h2_linear = (Mb * x + bb)';   % multiplies u later

    % h3 terms
    % Expand: (Ax + Bu)' * Qb * (Ax + Bu)
    H3_quad = (B' * Qb * B) + (B' * Mb');                         % u' H3_quad u
    H3_linear = (2 * Ax' * Qb * B) + (lb' * B) + ...   % (linear in u)
                (Mb * Ax)' + bb';                 % from (Mb*y + bb)' * u

    % Final cost: h2 - h3
    H = -(H3_quad);            % Only from h3 (since h2 is linear in u) RMEOVED THE 0.5*
    f = (h2_linear - H3_linear);      % sign flip since it's h2 - h3

    % Call quadprog
    options = optimoptions('quadprog', 'Display', 'off');
    [u_opt, val] = quadprog(2*H, f', [], [], [], [], u_min, u_max, [], options);

    % % Final value = h2 - h3
    % y = A_d * x + B_d * u_opt;
    % h2 = (x' * Qb * x) + (lb' * x) + (c) + ((Mb * x + bb)' * u_opt);
    % h3 = (y' * Qb * y) + (lb' * y) + (c) + ((Mb * y + bb)' * u_opt);
    % nu_val = h2 - h3;
    % 
    % val
    

end