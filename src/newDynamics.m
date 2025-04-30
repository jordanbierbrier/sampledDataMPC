function dx = newDynamics(x,u)

% x = (x, y, xD, yD, roll, pitch, rollD, pitchD)
% u = (t_roll, t_pitch)


A = [
    0 0 1 0 0 0 0 0 
    0 0 0 1 0 0 0 0
    0 0 0 0 0 9.81 0 0
    0 0 0 0 -9.81 0 0 0
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
    1/0.0093 0
    0 1/0.0092
    ];

dx = A*x + B*u;