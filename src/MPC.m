classdef MPC

    properties
        ns  % number of states
        ni  % number of inputs
        Ts 
        uMin
        uMax % change this
        N
        A_hat
        B_hat
        Q_bar
        R_bar
        H
        zlb
        zub
        Qb
        lb
        cb
        Mb
        bb
        u1_selector
        u2_selector
        Qb1
        lb1
        cb1
        Mb1
        bb1
        Qb2
        lb2
        cb2
        Mb2
        bb2
    end

    methods
        
        %%%%%%%%%%%%%%%%%%%% METHOD 1 %%%%%%%%%%%%%%%%%%%%
        function obj = MPC(A, B, Q, R, uMin, uMax, N, ns, ni, Ts, Qb,lb,cb,Mb,bb, Qb1,lb1,cb1,Mb1,bb1,Qb2,lb2,cb2,Mb2,bb2)

            obj.Ts = Ts;
            obj.ns = ns;
            obj.ni = ni;
            obj.N = N;

            obj.u1_selector = mod((1:obj.ni*obj.N)',2);
            obj.u2_selector = mod((0: obj.ni*obj.N - 1)',2);

            obj.Qb = Qb;
            obj.lb = lb;
            obj.cb = cb;
            obj.Mb = Mb;
            obj.bb = bb;

            obj.Qb1 = Qb1;
            obj.lb1 = lb1;
            obj.cb1 = cb1;
            obj.Mb1 = Mb1;
            obj.bb1 = bb1;

            obj.Qb2 = Qb2;
            obj.lb2 = lb2;
            obj.cb2 = cb2;
            obj.Mb2 = Mb2;
            obj.bb2 = bb2;

            I = eye(ns);
            obj.A_hat = I;
            for k = 1:N
                obj.A_hat = [obj.A_hat; A^k];
            end
            
            obj.B_hat = zeros((obj.N+1)*ns, ni);
            
            for i = 1:obj.N+1
                for j = 1:obj.N
                    if i > j
                        obj.B_hat((i-1)*ns+1:i*ns, (j-1)*ni+1:j*ni) = A^(i-j-1) * B;
                    end
                end
            end
            
            
            
            Q_kron = eye((obj.N+1));
            R_kron = eye(obj.N);
            
            obj.Q_bar = kron(Q_kron, Q);
            obj.R_bar = kron(R_kron, R);
            
            obj.H = (obj.B_hat' * obj.Q_bar * obj.B_hat) + obj.R_bar;
            
            
            %%%%%%% Optimization varaible bounds (i.e. for z) %%%%%%%%%%
            
            obj.zlb = uMin;
            obj.zub = uMax;
            
            for i = 1:(N-1)
                obj.zlb = [obj.zlb; uMin];
                obj.zub = [obj.zub; uMax];
            end
            
        end



        


        %%%%%%%%%%%%%%%%%%%% METHOD 3 %%%%%%%%%%%%%%%%%%%%
        function [z, update_count] = solveMPC(obj, x0, XRef_MPC, u_hil1, u_hil2, update_count, margin, margin1, margin2)
        
            options = optimoptions('quadprog','Display','off');
        
            MAX_COUNT = 10;
        
            G = zeros(1,(obj.N)*obj.ni);
            
            G(1,1:2) = -((obj.Mb*x0) + obj.bb).'; % Negative because --> Gu + g >= 0 ... g >= -Gu
            
            g = (x0.'*obj.Qb*x0) + (obj.lb.'*x0) + obj.cb + margin;


            G1 = zeros(1,(obj.N)*obj.ni);
            
            G1(1,1:2) = -((obj.Mb1*x0) + obj.bb1).'; % Negative because --> Gu + g >= 0 ... g >= -Gu

            g1 = (x0.'*obj.Qb1*x0) + (obj.lb1.'*x0) + obj.cb1 + margin1;


            G2 = zeros(1,(obj.N)*obj.ni);
            
            G2(1,1:2) = -((obj.Mb2*x0) + obj.bb2).'; % Negative because --> Gu + g >= 0 ... g >= -Gu

            g2 = (x0.'*obj.Qb2*x0) + (obj.lb2.'*x0) + obj.cb2 + margin2;
            
            %%%%%%%% for debugging purposes %%%%%%%%%%%%
            % Gtot = [Gtot;G(:,1:2)];
            % gtot = [gtot;g];
            % X0 = [X0 x0];
        
        
            z_hil = u_hil1 * obj.u1_selector + u_hil2 * obj.u2_selector;
            
        
            if update_count < MAX_COUNT
                h_trans = (((obj.A_hat * x0) - XRef_MPC)' * obj.Q_bar' * obj.B_hat) - (obj.R_bar*z_hil).';
                update_count = update_count + 1;
            else
                h_trans = ((obj.A_hat * x0) - XRef_MPC)' * obj.Q_bar' * obj.B_hat;
                update_count = MAX_COUNT;
            end 
            
            %%%%% second part of objective function... h_trans.'*z
            
        
            %%%%%%% Optimization Call %%%%%%%%%%%%%%%%
            % z = quadprog(obj.H, h_trans', [G; G1; G2], [g; g1; g2], [],[], obj.zlb, obj.zub, [], options);
            % z = quadprog(obj.H, h_trans', [G; G1], [g; g1], [],[], obj.zlb, obj.zub, [], options);
            z = quadprog(obj.H, h_trans', [G], [g], [],[], obj.zlb, obj.zub, [], options);
            % z = quadprog(obj.H, h_trans', [], [], [],[], obj.zlb, obj.zub, [], options);
        end

    end
end