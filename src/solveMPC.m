function [z, update_count] = solveMPC(H, A_hat, B_hat, Q_bar, R_bar, x0, XRef_MPC, Qb, lb, cb, Mb, bb, zlb, zub, N, ni, u_hil1, u_hil2, update_count)

    options = optimoptions('quadprog','Display','off');

    MAX_COUNT = 7;

    G = zeros(1,(N)*ni);
    
    G(1,1:2) = -((Mb*x0) + bb).'; % Negative because --> Gu + g >= 0 ... g >= -Gu

    g = (x0.'*Qb*x0) + (lb.'*x0) + cb; 
    
    %%%%%%%% for debugging purposes %%%%%%%%%%%%
    % Gtot = [Gtot;G(:,1:2)];
    % gtot = [gtot;g];
    % X0 = [X0 x0];


    z_hil = u_hil1 * mod((1:ni*N)',2) + u_hil2 * mod((0: ni*N - 1)',2);
    

    if update_count < MAX_COUNT
        h_trans = (((A_hat * x0) - XRef_MPC)' * Q_bar' * B_hat) - (R_bar*z_hil).';
        update_count = update_count + 1;
        update_count
    else
        h_trans = ((A_hat * x0) - XRef_MPC)' * Q_bar' * B_hat;
        update_count = MAX_COUNT;
    end 
    %%%%% second part of objective function... h_trans.'*z
    


    %%%%%%% Optimization Call %%%%%%%%%%%%%%%%
    z = quadprog(H, h_trans', G, g, [],[], zlb, zub, [], options);
end