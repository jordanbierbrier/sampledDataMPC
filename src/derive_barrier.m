clc
clear
close all

syms t g Ix Iy R xC yC lam1 lam2 lam3 lam4

p = sym('p',[2,1]); % position
v = sym('v',[2,1]); % linear velocity
a = sym('a',[2,1]); % Euler angles
w = sym('w',[2,1]); % angular velocity

x = [p;v;a;w]; % state vector
u = sym('u',[2,1]); % control input vector

center = [xC; yC]; % center of obstacle

A = [
    0 0 1 0 0 0 0 0 
    0 0 0 1 0 0 0 0
    0 0 0 0 0 g 0 0
    0 0 0 0 -g 0 0 0
    0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 1 
    0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0
    ];


B = [
    0 0
    0 0
    0 0
    0 0
    0 0
    0 0
    1/Ix 0
    0 1/Iy
    ];


% dynamics
x_dot = simplify(A*x + B*u);

% Initial barrier constaint
h0 = (p - center).'*(p - center) - R^2;

dh0dt = jacobian(h0, t);

h0_dot = jacobian(h0, x)*x_dot + dh0dt;

% h1 positive (or zero) ensures h0 remains positive
h1 = h0_dot + lam1*h0;

dh1dt = jacobian(h1, t);

h1_dot = jacobian(h1,x)*x_dot + dh1dt;


% h2 positive (or zero) ensures h1 remains positive
h2 = h1_dot + lam2*h1;

dh2dt = jacobian(h2, t);

h2_dot = jacobian(h2,x)*x_dot + dh2dt;

% h3 positive (or zero) ensures h2 remains positive
h3 = h2_dot + lam3*h2;

dh3dt = jacobian(h3, t);

h3_dot = jacobian(h3,x)*x_dot + dh3dt;

% h4 positive (or zero) ensures h3 remains positive

h4 = h3_dot + lam4*h3;


test = h4;

[Q,l,c,M,b] = barrierMatrixForm(test,x,u);
simplify(x.'*Q*x + l.'*x + c + (M*x + b).'*u - test)

Q
l
c
M
b