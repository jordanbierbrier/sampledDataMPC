function [A_hat, B_hat, Q_bar, R_bar, H, zlb, zub] = setupMPC(A, B, Q, R, uMin, uMax, N, ns, ni)
%SETUPMPC Summary of this function goes here
%   Detailed explanation goes here

%%%%%% Setup H for QP... (1/2)z.'*H*z within obj fn, where z = [u,...,uN] %%%%%%%%%%%
    I = eye(ns);
    A_hat = I;
    for k = 1:N
        A_hat = [A_hat; A^k];
    end
    
    B_hat = zeros((N+1)*ns, ni);
    
    for i = 1:N+1
        for j = 1:N
            if i > j
                B_hat((i-1)*ns+1:i*ns, (j-1)*ni+1:j*ni) = A^(i-j-1) * B;
            end
        end
    end
    
    
    
    Q_kron = eye((N+1));
    R_kron = eye(N);
    
    Q_bar = kron(Q_kron, Q);
    R_bar = kron(R_kron, R);
    
    H = (B_hat' * Q_bar * B_hat) + R_bar;
    
    
    %%%%%%% Optimization varaible bounds (i.e. for z) %%%%%%%%%%
    
    zlb = uMin;
    zub = uMax;
    
    for i = 1:(N-1)
        zlb = [zlb; uMin];
        zub = [zub; uMax];
    end
end

