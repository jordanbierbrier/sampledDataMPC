function xout = rk4_integrator(fun, dt, xk, uk)

f1 = fun(xk, uk);
f2 = fun((xk + (dt/2)*f1), uk);
f3 = fun((xk + (dt/2)*f2), uk);
f4 = fun((xk + dt*f3), uk);

xout = xk + (dt/6)*(f1 + 2*f2 + 2*f3 + f4);
end