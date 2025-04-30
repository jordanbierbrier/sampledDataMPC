function [Qb,lb,cb,Mb,bb] = buildBarrier(vals)

syms g Ix Iy R xC yC lam1 lam2 lam3 lam4

p = sym('p',[2,1]); % position
v = sym('v',[2,1]); % linear velocity
a = sym('a',[2,1]); % Euler angles
w = sym('w',[2,1]); % angular velocity

x = [p;v;a;w]; % state vector
u = sym('u',[2,1]); % control input vector

vars = [g Ix Iy R xC yC lam1 lam2 lam3 lam4];

barrier_to_use = deriveBarrier();

[Q,l,c,M,b] = barrierMatrixForm(barrier_to_use,x,u);

Qb = double(subs(Q, vars, vals));
lb = double(subs(l, vars, vals));
cb = double(subs(c, vars, vals));
Mb = double(subs(M, vars, vals));
bb = double(subs(b, vars, vals));

end

