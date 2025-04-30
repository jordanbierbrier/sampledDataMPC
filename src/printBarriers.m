% % clc
% % close
% % clear all
% 
% %%%%%%% Constants %%%%%%%%%%%%
% ns = 8;  % number of states
% ni = 2;  % number of inputs
% Ts = 0.01;  % sampling time
% T = 10;  % total sim time (seconds)
% time = 0:Ts:T;
% [rows_time, total_refs] = size(time);
% 
% x_ref = zeros(1, total_refs);
% y_ref = zeros(1, total_refs);
% 
% x_ref(1) = 0;  % Initial position x(0)
% y_ref(1) = 0;  % Initial position y(0)
% 
% for k = 2:total_refs
%     v_x = -5 * sin(time(k-1)); % Compute velocity at previous step
%     v_y =  5 * cos(time(k-1));
% 
%     % Euler integration for position
%     x_ref(k) = x_ref(k-1) + v_x * Ts;
%     y_ref(k) = y_ref(k-1) + v_y * Ts;
% end
% 
% 
% 
% g = 9.81;
% Ix = 0.0093;
% Iy = 0.0092;
% 
% %%%%%%%%%% Obstacle constants %%%%%%%
% xC = -7.;
% yC = -3.5;
% RR = 1.; % radius of obstacle
% 
% A = [
%     0 0 1 0 0 0 0 0 
%     0 0 0 1 0 0 0 0
%     0 0 0 0 0 g 0 0
%     0 0 0 0 -g 0 0 0
%     0 0 0 0 0 0 1 0
%     0 0 0 0 0 0 0 1 
%     0 0 0 0 0 0 0 0 
%     0 0 0 0 0 0 0 0
%     ];
% 
% B = [
%     0 0
%     0 0
%     0 0
%     0 0
%     0 0
%     0 0
%     1/Ix 0
%     0 1/Iy
%     ];
% 
% 
% sys = ss(A, B, [], []);
% sys_d = c2d(sys, Ts, 'zoh');
% 
% Ad = sys_d.A;
% Bd = sys_d.B;
% 
% 
% x0_cont = [0;0;0;0;0;0;0;0];
% x0_dis = [0;0;0;0;0;0;0;0];
% 
% x0_wrong = [0;0;0;0;0;0;0;0];
% X_wrong = x0_wrong;
% 
% X_cont = x0_cont;
% X_dis = x0_dis;
% 
% for i = 1:length(U1)
%     in = [U1(i);U2(i)];
%     xcont_next = x0_cont+ (A*x0_cont + B*in)*Ts;
%     xdis_next = Ad*x0_dis + Bd*in;
% 
%     xwrong_next = A*x0_wrong + B*in;
%     X_wrong = [X_wrong xwrong_next];
% 
%     X_cont = [X_cont xcont_next];
%     X_dis = [X_dis xdis_next];
% 
%     x0_cont = xcont_next;
%     x0_dis = xdis_next;
% 
%     x0_wrong = xwrong_next;
% end
% 
% x_out_cont = X_cont(1,:);
% y_out_cont = X_cont(2,:);
% 
% x_out_dis = X_dis(1,:);
% y_out_dis = X_dis(2,:);
% 
% x_out_wrong = X_wrong(1,:);
% y_out_wrong = X_wrong(2,:);
% 
% figure;
% plot(x_out_cont, y_out_cont, 'LineWidth',2);
% hold on
% plot(x_out_dis, y_out_dis, 'LineWidth',2);
% hold on
% plot(x_out_wrong, y_out_wrong, 'LineWidth',2);
% hold on




% % Sample animated trajectory
% figure;
% hold on;
% axis([0 10 0 10]); % Set axis limits
% grid on;
% 
% % Simulated trajectory
% x = linspace(0, 10, 100);
% y = sin(x);
% 
% for k = 1:length(x)
%     plot(x(k), y(k), 'ro', 'MarkerFaceColor', 'r'); % Plot current point
%     drawnow; % Force immediate update
%     pause(0.05); % Control speed of animation
% end


%%%%%%%%%%% Second Implementation %%%%%%%%%%%%%%

% figure;
% hold on;
% axis([0 10 -1 1]);
% grid on;
% 
% h = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r'); % Initialize plot object
% 
% for k = 1:length(x)
%     set(h, 'XData', x(k), 'YData', y(k)); % Update plot data
%     drawnow;
%     pause(0.05);
% end

%%%%%%%%%%% Third Implementation %%%%%%%%%%%%%

% figure;
% hold on;
% axis([0 10 -1 1]);
% grid on;
% 
% h = animatedline('Color', 'b', 'LineWidth', 2);
% 
% for k = 1:length(x)
%     addpoints(h, x(k), y(k));
%     drawnow;
%     pause(0.05);
% end




%%%%%%%%% Fourth Implementation %%%%%%%%%%%%%%%
% 
% 
% function interactive_plot()
%     % Create figure
%     fig = figure;
%     hold on;
%     axis([0 20 -10 10]);
%     grid on;
% 
%     % Initial position
%     x = 5; 
%     y = 0;
% 
%     % Plot point
%     h = plot(x, y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
% 
%     % Set keypress callback function
%     set(fig, 'KeyPressFcn', @(src, event) key_callback(event, h));
% 
%     % Simulation loop (dummy loop for continuous update)
%     for t = 1:100
%         drawnow;
%         pause(0.1);
%     end
% end
% 
% % Callback function for key presses
% function key_callback(event, h)
%     % Get current position
%     x = get(h, 'XData');
%     y = get(h, 'YData');
% 
%     % Adjust position based on key press
%     switch event.Key
%         case 'rightarrow'
%             x = x + 1;
%         case 'leftarrow'
%             x = x - 1;
%         case 'uparrow'
%             y = y + 1;
%         case 'downarrow'
%             y = y - 1;
%     end
% 
%     % Update plot
%     set(h, 'XData', x, 'YData', y);
% end
% 
% 
%     % h = figure; 
%     % set(h,'KeyPressFcn',@KeyPressCb);
%     % function y = KeyPressCb(~,evnt)
%     %     fprintf('key pressed: %s\n',evnt.Key);
%     %     global s;
%     %     if strcmp(evnt.Key,'rightarrow')==1
%     %     s = evnt.Key;
%     %     elseif strcmp(evnt.Key, 'leftarrow')==1
%     %     s = evnt.Key;
%     %     elseif strcmp(evnt.Key,'space')==1
%     %     s = evnt.Key;    
%     %     end
%     % end








function interactive_trajectory()
    % Create figure
    fig = figure;
    hold on;
    axis([0 10 -2 2]); % Set axis limits
    grid on;

    % Initial trajectory
    x = linspace(0, 10, 200);
    y = sin(x);
    user_offset = 0; % User-controlled vertical offset

    % Plot elements
    h_trajectory = plot(x, y, 'b', 'LineWidth', 2); % Static trajectory
    h_point = plot(x(1), y(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10); % Moving point

    % Set keypress callback function
    set(fig, 'KeyPressFcn', @(src, event) key_callback(event));

    % Main loop (updates position dynamically)
    for k = 1:length(x)
        % Update point's position based on trajectory + user input
        set(h_point, 'XData', x(k), 'YData', y(k) + user_offset);
        
        drawnow; % Update figure
        pause(0.05); % Control speed of animation
    end

    % Nested function for key events (non-blocking input)
    function key_callback(event)
        switch event.Key
            case 'uparrow'
                user_offset = user_offset + 0.1; % Move up
            case 'downarrow'
                user_offset = user_offset - 0.1; % Move down
        end
    end
end