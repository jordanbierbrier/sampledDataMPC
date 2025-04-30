function xout = rk4_lin(A, B, dt, xk, uk)
    % Compute RK4 integration steps
    f1 = A*xk + B*uk;
    f2 = A*(xk + (dt/2)*f1) + B*uk;
    f3 = A*(xk + (dt/2)*f2) + B*uk;
    f4 = A*(xk + dt*f3) + B*uk;
    
    % Final RK4 integration step
    xout = xk + (dt/6)*(f1 + 2*f2 + 2*f3 + f4);
end
