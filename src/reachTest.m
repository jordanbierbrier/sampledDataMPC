clc
close
clear all

% Constants
g = 9.81;
Ix = 0.0093;
Iy = 0.0092;

% Continuous-time system matrices
Ac = [
    0 0 1 0 0 0 0 0 
    0 0 0 1 0 0 0 0
    0 0 0 0 0 g 0 0
    0 0 0 0 -g 0 0 0
    0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 1 
    0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0
    ];

Bc = [
    0 0
    0 0
    0 0
    0 0
    0 0
    0 0
    1/Ix 0
    0 1/Iy
    ];

% Discretization
Ts = 0.02;  % sampling time in seconds
sys_c = ss(Ac, Bc, eye(8), zeros(8,2));
sys_d = c2d(sys_c, Ts);

Ad = sys_d.A;
Bd = sys_d.B;

% Initial state
% x0 = zeros(8,1);
x0 = [-3.58168907268387
-4.79462137331569
2.39907358284142
0.703159244616636
0.122366028601254
0.0355330603115410
0
0];

% Input bounds
u_min = [-1; -1];
u_max = [1; 1];


% Step 1: Setup (run once)
reach_data = setupReachability(Bd, u_min, u_max);

% Step 2: Query reachable set from different states
x1 = zeros(8,1);
[H1, h1] = reachableSet(Ad, x1, reach_data);


[H2, h2] = reachableSet(Ad, x0, reach_data);
% After computing H and h:
P = Polyhedron('A', H2, 'b', h2);

% Plot a 2D projection (e.g., x1 vs x2)
figure;
plot(P.projection([1, 2]), 'color', [0.6 0.8 1]);
xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('Reachable Set: $x_1$ vs $x_2$', 'Interpreter', 'latex');
grid on; axis equal;