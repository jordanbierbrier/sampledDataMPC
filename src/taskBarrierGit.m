clc
close
clear all

%%%%%%% Constants %%%%%%%%%%%%
ns = 8;  % number of states
ni = 2;  % number of inputs
Ts = 0.01;  % sampling time
T = 10;  % total sim time (seconds)
time = 0:Ts:T;
[rows_time, total_refs] = size(time);

g = 9.81;
Ix = 0.0093;
Iy = 0.0092;

lam1 = 9;
lam2 = 9;
lam3 = 9;
lam4 = 9;

global u1_hil u2_hil UPDATE_COUNT;
UPDATE_COUNT = 100; 
u1_hil = 0;
u2_hil = 0;

%%%%%%%%%% Obstacle constants %%%%%%%

% xC = -1.9;
% yC = 3.;
% RR = .5; % radius of obstacle

xC = -1.6;
yC = 3.;
RR = .5; % radius of obstacle

xC1 = -3.6;
yC1 = 4.2;
RR1 = .0; % radius of obstacle

xC2 = -5.;
yC2 = 5.5;
RR2 = .0; % radius of obstacle


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

Ad = sys_d.A;
Bd = sys_d.B;

A = Ad;
B = Bd;


% reference path to follow. Path starts at position (0,0)
omega = 1;
radius = 5;
x_traj = radius * cos(omega * time) - radius;
y_traj = radius * sin(omega * time);
x_vel_traj = -omega*radius * sin(omega * time);
y_vel_traj = omega*radius * cos(omega * time);






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Cutoff angle (pi/2 = 90 degrees)
theta_cutoff = pi/2;
t_cutoff = theta_cutoff / omega;
idx_cutoff = find(time >= t_cutoff, 1);

% First part: circular arc
theta_arc = omega * time(1:idx_cutoff);
x_arc = radius * cos(theta_arc) - radius;
y_arc = radius * sin(theta_arc);

x_vel_arc = -omega * radius * sin(theta_arc);
y_vel_arc = omega * radius * cos(theta_arc);

% Second part: straight leftward motion
% Duration or distance
duration_straight = 5; % seconds
t_line = 0:Ts:duration_straight;

v_x_straight = x_vel_arc(end);  % maintain x velocity
v_y_straight = 0;

x_line = x_arc(end) + v_x_straight * t_line;
y_line = y_arc(end) * ones(size(t_line));

x_vel_line = v_x_straight * ones(size(t_line));
y_vel_line = v_y_straight * ones(size(t_line));

% Combine arc and straight path
x_traj = [x_arc, x_line];
y_traj = [y_arc, y_line];
x_vel_traj = [x_vel_arc, x_vel_line];
y_vel_traj = [y_vel_arc, y_vel_line];

total_refs = length(x_traj);  % match new reference length

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%% Barrier Terms in Matrix Form %%%%%%%%%%%
% i.e. h4 = x.'*Qb*x + lb.'*x + c + (Mb*x + bb).'*u  >= 0 (g >= -Gu)
% h4 is affine in u. Also note h4:= h3_dot + lam(h3) >= 0
constant_vals = [g Ix Iy RR xC yC lam1 lam2 lam3 lam4];

[Qb,lb,cb,Mb,bb] = buildBarrier(constant_vals);


constant_vals1 = [g Ix Iy RR1 xC1 yC1 lam1 lam2 lam3 lam4];
[Qb1,lb1,cb1,Mb1,bb1] = buildBarrier(constant_vals1);


constant_vals2 = [g Ix Iy RR2 xC2 yC2 lam1 lam2 lam3 lam4];
[Qb2,lb2,cb2,Mb2,bb2] = buildBarrier(constant_vals2);

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! %
all_barriers = deriveAllBarriers(constant_vals); % this is used to print all barrier vals
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! %

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Costs
Q = diag([10, 10, 2, 2, 1, 1, 0, 0]); % state deviation costs
R = .5 * eye(ni); % input costs

% Controller constraints
global uMin uMax;
uMin = [-1.; -1.];
uMax = [1.; 1.];



%%%%%%%%%%%%%%%%%% MARGIN CALCULATION %%%%%%%%%%%%%%%%%%
% xX = [-3.5816; -4.79462; 2.3990; 0.7031592; 0.1223660; 0.0355330; 0; 0];
% [nu_val, u_opt] = findMargin(A, B, xX, Qb, lb, Mb, bb, cb, uMin, uMax)
% return
%%%%%%%%%%%%%%%%%% MARGIN CALCULATION %%%%%%%%%%%%%%%%%%


% Reasonable yaw/pitch angles over time
theta_x = 0.1 * sin(2*pi*time);   % radians faster, larger motion
theta_y = 0.1 * cos(2*pi*time);

% Integrate to get linear acceleration:
% vx_dot = g * theta_y;
% vy_dot = -g * theta_x;

% vx = cumtrapz(time, vx_dot);
% vy = cumtrapz(time, vy_dot);

% x_pos = cumtrapz(time, vx);
% y_pos = cumtrapz(time, vy);

vx_dot = 0.2*pi * cos(2*pi*time);
vy_dot = -0.2*pi * sin(2*pi*time);

vx = -g*0.2*pi * sin(2*pi*time);
vy = g*0.2*pi * cos(2*pi*time);

x_pos = -g*0.2*pi * cos(2*pi*time)*(2*pi);
y_pos = -g*0.2*pi * sin(2*pi*time)*(2*pi);


% Integrate again to get velocities and positions


r = 5;        % radius of circle
omega = 0.5;  % angular speed of circular motion

x_pos = r * cos(omega * time) - r;
y_pos = r * sin(omega * time);
vx    = -r * omega * sin(omega * time);
vy    =  r * omega * cos(omega * time);
ax    = -r * omega^2 * cos(omega * time);
ay    = -r * omega^2 * sin(omega * time);

% Approximate desired pitch and roll angles (small angle assumption)
% Newton's second law: F = m*a => desired angle = a/g
% theta_x (roll): needed for lateral acceleration (along y)
% theta_y (pitch): needed for forward acceleration (along x)

theta_x = ay / g;    % roll
theta_y = -ax / g;   % pitch




time = 0:Ts:T;

x_ref = zeros(8, length(time));
x_ref(1, :) = x_pos;
x_ref(2, :) = y_pos;
x_ref(3, :) = vx;
x_ref(4, :) = vy;
x_ref(5, :) = theta_x;
x_ref(6, :) = theta_y;
% x_ref(7, :) = omega_x;
% x_ref(8, :) = omega_y;


XRef = reshape(x_ref, [], 1);


% % States reference
XRef = [x_traj', y_traj',x_vel_traj',y_vel_traj' zeros(total_refs, 4)];
XRef = reshape(XRef', ns*total_refs, 1);

N = 20; % horizon 
% steps = 1300; % number of sim steps to take
steps = 196; % 200 for one obstacle
% steps = 220; % 220 for two obstacles
% steps = 240; % 240 for three obstacles
noise_std = 0; %0.001; % change to add noise to model during update

x0 = XRef(1:ns); % initial state
X = x0; % state vectors of simulator

% saving all control inputs 
U1 = [];
U2 = [];


% for debugging, saving barrier constraint matrices i.e. -Gu <= g
Gtot = [];
gtot = [];


refLen = length(XRef);


% [A_hat, B_hat, Q_bar, R_bar, H, zlb, zub] = setupMPC(A, B, Q, R, uMin, uMax, N, ns, ni);
mpcObj = MPCGIT(A, B, Q, R, uMin, uMax, N, ns, ni, Ts, Qb,lb,cb,Mb,bb, Qb1,lb1,cb1,Mb1,bb1, Qb2,lb2,cb2,Mb2,bb2);


%%%%%%%%%%%%% OUTPUT FIGURE %%%%%%%%%%%%%%%%%%%%%%%
fig = figure;
hold on;
% axis([-10 0 -6 6]); % Set axis limits
axis([-7 .5 -.5 7]); % Set axis limits
grid on;

Xp = reshape(X, [ns,1]);
x_out = Xp(1,:);
y_out = Xp(2,:);

% put + 20 for two obstacles                                        
ref_plot = plot(x_traj(1:steps - N + 4), y_traj(1:steps - N + 4), 'LineWidth',2, 'Color','r');
% plot(x_pos, y_pos, 'LineWidth',2, 'Color','r');

theta = linspace(0,2*pi,100);
obs_x = xC + RR*cos(theta);
obs_y = yC + RR*sin(theta);
obs_plot = plot(obs_x, obs_y, 'LineWidth',2, 'Color','y');

theta = linspace(0,2*pi,100);
obs_x1 = xC1 + RR1*cos(theta);
obs_y1 = yC1 + RR1*sin(theta);
obs1_plot = plot(obs_x1, obs_y1, 'LineWidth',2, 'Color','y');

theta = linspace(0,2*pi,100);
obs_x2 = xC2 + RR2*cos(theta);
obs_y2 = yC2 + RR2*sin(theta);
obs2_plot = plot(obs_x2, obs_y2, 'LineWidth',2, 'Color','y');

%%%%%%%%%%%%%%

axis_length = 0.5;  % arrow length

% Initialize fixed-direction quivers (global axes)
q_x = quiver(0, 0, axis_length, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);
q_y = quiver(0, 0, 0, axis_length, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 2);
q_x.HandleVisibility = 'off';
q_y.HandleVisibility = 'off';

%%%%%%%%%%%%%%

output_mpc = plot(x_out, y_out, 'LineWidth', 2, 'Color','b');

set(fig, 'KeyPressFcn', @(src, event) key_callback(event));

legend([ref_plot, output_mpc, obs_plot], {'reference', 'trajectory', 'obstacles'})


%%%%% Barrier figures - BEGIN %%%%%%
M = [];
H0 = [];
H1 = [];
H2 = [];
H3 = [];
H4 = [];


% figure;
% hold on;
% axis([0 steps -1 80]);
% h0_plot = plot(1:1, 0, "LineWidth", 2);

%%%%% Barrier figures - END %%%%%%


% loop for MPC
for step = 1:steps


    XRef_MPC = getReference(XRef, step, N, ns, refLen);

    % [z, UPDATE_COUNT] = solveMPC(H, A_hat, B_hat, Q_bar, R_bar, x0, XRef_MPC, Qb, lb, cb, Mb, bb, zlb, zub, N, ni, u1_hil, u2_hil, UPDATE_COUNT);
    
    margin = 0;
    [margin, u_opt] = findMargin(A, B, x0, Qb, lb, Mb, bb, cb, uMin, uMax);
    M = [M; margin];
    
    margin1 = 0;
    [margin1, u_opt1] = findMargin(A, B, x0, Qb1, lb1, Mb1, bb1, cb1, uMin, uMax);

    margin2 = 0;
    [margin2, u_opt2] = findMargin(A, B, x0, Qb2, lb2, Mb2, bb2, cb2, uMin, uMax);


    [z, UPDATE_COUNT] = mpcObj.solveMPC(x0, XRef_MPC, u1_hil, u2_hil, UPDATE_COUNT, margin, margin1, margin2);

    noise = randn(ns,1) * noise_std; % currently adding no noise
    x_next = A*x0 + B*z(1:2) + noise; % propagate system forward


    % barrier_vals = barrierValues(all_barriers, x0, z(1:2));
    % H0 = [H0; barrier_vals(1)];

    %%%%%% saving inputs and states %%%%%%%%
    U1 = [U1;z(1)];
    U2 = [U2;z(2)];
    X = [X;x_next];

    Xp = reshape(X, [ns,step + 1]);
    x_out = Xp(1,:);
    y_out = Xp(2,:);
     
    
    % set(u1_plot, 'XData',1:length(U1), 'YData', U1);
    % set(u2_plot, 'XData',1:length(U2), 'YData', U2);
    set(output_mpc, 'XData', x_out, 'YData', y_out);

    % Get current tip position
    x_now = x_out(end);
    y_now = y_out(end);
    
    % Move arrows — same direction, new origin
    set(q_x, 'XData', x_now, 'YData', y_now);
    set(q_y, 'XData', x_now, 'YData', y_now);
    

    %%%%%% Barrier Plots - Begin %%%%%%

    % set(h0_plot, 'XData', 1:length(H0), 'YData', H0)
    

    %%%%%% Barrier Plots - Start %%%%%%
    


    drawnow;
    % pause(Ts/100); % Speed of animation
    
    x0 = x_next;
end



H0 = zeros(steps-1, 1);
H1 = zeros(steps-1, 1);
H2 = zeros(steps-1, 1);
H3 = zeros(steps-1, 1);
H4 = zeros(steps-1, 1);

for i=1:steps-1
    
    
    idx = ((i-1)*8) + 1;
    xx = X(idx:idx+7,1);
    z = [U1(i); U2(i)];
    barrier_vals = barrierValues(all_barriers, xx, z);


    H0(i) = barrier_vals(1);
    H1(i) = barrier_vals(2);
    H2(i) = barrier_vals(3);
    H3(i) = barrier_vals(4);
    H4(i) = barrier_vals(5) + M(i);
    
end

tol = 1e-9;  % Desired tolerance

% After loop:
H0(abs(H0) < tol) = 0;
H1(abs(H1) < tol) = 0;
H2(abs(H2) < tol) = 0;
H3(abs(H3) < tol) = 0;
H4(abs(H4) < tol) = 0;

min(H0)
min(H1)
min(H2)
min(H3)
min(H4)

figure;
plot(H0)
figure;
plot(H1)
figure;
plot(H2)
figure;
plot(H3)
figure;
plot(H4)
% set(h0_plot, 'XData', 1:length(H0), 'YData', H0)

figure;
set(u1_plot, 'XData',1:length(U1), 'YData', U1);
set(u2_plot, 'XData',1:length(U2), 'YData', U2);


function key_callback(event)
    global u1_hil u2_hil UPDATE_COUNT uMin uMax;
    UPDATE_COUNT = 0;
    switch event.Key 
        case 'uparrow'
            u1_hil = uMax(1)*1.;
            u2_hil = 0;
        case 'downarrow'
            u1_hil = uMin(1)*1.;
            u1_hil
            u2_hil = 0;
        case 'rightarrow'
            u1_hil = 0;
            u2_hil = uMax(2)*1.;
        case 'leftarrow'
            u1_hil = 0;
            u2_hil = uMin(2)*1.;
    end
end


% figure;
% plot(U1, "LineWidth", 2);
% hold on
% plot(U2, "LineWidth", 2);
% hold on
% yline(uMin, "LineWidth", 2)
% yline(uMax, "LineWidth", 2)
% legend("u1", "u2")

%%%%%% checks if h4 >= 0 is always satisfied (i.e. -Gu <= g) %%%%%
% GIneq = sum(Gtot.*[U1 U2], 2);
% conOut = GIneq <= gtot;
% if any(conOut == 0)
%     disp("h4 Constraint Not Satisfied");
% else
%     disp("Satisfied h4 Constraint");
% end


% X = reshape(X, [ns,steps + 1]);
% 
% x_out = X(1,:);
% y_out = X(2,:);
% vx_out = X(3,:);
% vy_out = X(4,:);
% roll_out = rad2deg(X(5,:));
% pitch_out = rad2deg(X(6,:));


% figure;
% plot(x_out, y_out, 'LineWidth',2);
% hold on
% plot(x_traj, y_traj, 'LineWidth',2);
% hold on
% 
% theta = linspace(0,2*pi,100);
% obs_x = xC + RR*cos(theta);
% obs_y = yC + RR*sin(theta);
% plot(obs_x, obs_y, 'LineWidth',2);








% legend("Output", "Reference", "Obstacle Region")
% figure;
% plot(U1);
% hold on
% plot(U2);