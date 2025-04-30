function [Q, l, c, M, b] = barrierMatrixForm(h1, x, u)
    % barrierMatrixForm Decomposes h1 into quadratic, linear, and constant terms
    % Inputs:
    %   h1 - symbolic expression
    %   x  - symbolic state vector
    % Outputs:
    %   Q  - Quadratic coefficient matrix
    %   l  - Linear coefficient vector
    %   c  - Constant term
    
    num_vars = length(x);
    Q = sym(zeros(num_vars, num_vars));  % Quadratic term matrix
    l = sym(zeros(num_vars, 1));         % Linear term vector

    B = gradient(h1,u);
    B1 = B(1);
    B2 = B(2);

    M = [gradient(B1,x).'; gradient(B2,x).'];
    
    b = [simplify(B1 - M(1,:)*x); simplify(B2 - M(2,:)*x)];

    quad_term = h1 - B.'*u;
    
    % Compute the quadratic term matrix Q
    Q = hessian(quad_term, x)/2;

    linear_term = simplify(quad_term - (x.' * Q * x));
    
    l = gradient(linear_term, x);
    
    % Compute the constant term c
    c = simplify(h1 - x.' * Q * x - l.' * x - B.'*u);
end
