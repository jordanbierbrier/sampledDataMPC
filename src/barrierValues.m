function barrier_vals = barrierValues(barriers, x_val, u_val)

    
    
    
    p = sym('p',[2,1]); % position
    v = sym('v',[2,1]); % linear velocity
    a = sym('a',[2,1]); % Euler angles
    w = sym('w',[2,1]); % angular velocity
    
    x = [p;v;a;w]; % state vector
    u = sym('u',[2,1]); % control input vector

    vars = [x; u];
    vals = [x_val; u_val];

    h0_out = double(subs(barriers(1), vars, vals));
    h1_out = double(subs(barriers(2), vars, vals));
    h2_out = double(subs(barriers(3), vars, vals));
    h3_out = double(subs(barriers(4), vars, vals));
    h4_out = double(subs(barriers(5), vars, vals));

    barrier_vals = [h0_out, h1_out, h2_out, h3_out, h4_out];

end