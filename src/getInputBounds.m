clc
close
clear all

Ts = 0.01;  % sampling time
T = 10;  % total sim time (seconds)
time = 0:Ts:T;

g = 9.81;
Ix = 0.0093;
Iy = 0.0092;

v = 1;             % Linear x-velocity
A_sin = 10;         % Amplitude
omega = 2*pi;      % 1 Hz sine wave

x_pos = v * time;
y_pos = A_sin * sin(omega * time);

N = length(time);
% x_ref = zeros(8, N);
% x_ref(1, :) = x_pos;
% x_ref(2, :) = y_pos;

theta_x = 0.3 * sin(4*pi*time);   % faster, larger motion
theta_y = 0.3 * cos(4*pi*time);

% % Reasonable yaw/pitch angles over time
% theta_x = 0.1 * sin(2*pi*time);   % radians
% theta_y = 0.1 * cos(2*pi*time);

% Integrate to get linear acceleration:
vx_dot = g * theta_y;
vy_dot = -g * theta_x;

% Integrate again to get velocities and positions
vx = cumtrapz(time, vx_dot);
vy = cumtrapz(time, vy_dot);

x_pos = cumtrapz(time, vx);
y_pos = cumtrapz(time, vy);

x_ref = zeros(8, length(time));
x_ref(1, :) = x_pos;
x_ref(2, :) = y_pos;
x_ref(3, :) = vx;
x_ref(4, :) = vy;
x_ref(5, :) = theta_x;
x_ref(6, :) = theta_y;

tt = reshape(x_ref, [], 1);



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

sys = ss(Ac, Bc, [], []);
sys_d = c2d(sys, Ts, 'zoh');

A_d = sys_d.A;
B_d = sys_d.B;



u = zeros(2, N-1);  % 2 inputs over N-1 steps

for k = 1:N-1
    xk = x_ref(:, k);
    xk1 = x_ref(:, k+1);
    u(:, k) = pinv(B_d) * (xk1 - A_d * xk);
end

plot(time, x_ref(1,:), time, x_ref(2,:));
legend('x position','y position');
title('Reference Trajectory');

u_min = min(u, [], 2);
u_max = max(u, [], 2);

% Add 10% safety margin
margin = 0.1;
u_lower_bound = u_min .* (1 + sign(u_min) * margin);
u_upper_bound = u_max .* (1 + sign(u_max) * margin);

fprintf('Input bounds:\n');
fprintf('u1 ∈ [%.4f, %.4f]\n', u_lower_bound(1), u_upper_bound(1));
fprintf('u2 ∈ [%.4f, %.4f]\n', u_lower_bound(2), u_upper_bound(2));