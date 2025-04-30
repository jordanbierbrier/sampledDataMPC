function h = getBarrier(x,u,A,B)

xd = A*x + B*u;
tt = jacobian(xd,x)

h = 2;