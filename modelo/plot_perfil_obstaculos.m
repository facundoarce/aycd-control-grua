% % Extract time and data
% time_steps = out.y_c0.time;  % Time steps (4 values)
% data = squeeze(out.y_c0.signals.values);  % Reshape to [1651, 4]
% 
% % Generate x-axis (assuming x varies from 1 to 1651)
% x_min = x_t_min_oper - H_c/2;
% x_max = x_t_max_oper + H_c/2;
% x = x_min:0.05:x_max; % Define x positions
% 
% % Plot each time step as a separate curve
% plot(x, data(:,1), '-o', x, data(:,2), '-s', ...
%      x, data(:,3), '-d', x, data(:,4), '-^');
% 
% xlabel('x');
% ylabel('Variable y');
% title('Vector Evolution Over Time');
% legend(['t = ' num2str(time_steps(1))], ['t = ' num2str(time_steps(2))], ...
%        ['t = ' num2str(time_steps(3))], ['t = ' num2str(time_steps(4))]);
% grid on;

% Extract time steps dynamically
time_steps = out.y_c0.time;  % Size: [N, 1]
N = length(time_steps);  % Number of time steps

% Extract vector data (size: [1651, N])
data = squeeze(out.y_c0.signals.values);  

% Generate x-axis
x_min = x_t_min_oper - H_c/2;
x_max = x_t_max_oper + H_c/2;
x = x_min:0.05:x_max; % Define x positions

% Create figure
figure;
h = plot(x, data(:,1), 'LineWidth', 2); % Initial plot
xlabel('x');
ylabel('y');
title('Perfil de obstáculos');
grid on;

% Animate the evolution over time
for i = 1:N
    set(h, 'YData', data(:, i));  % Update the plot with new data
    title(['Perfil de obstáculos en t = ' num2str(time_steps(i)) ' s']);
    
    % Calculate time difference for accurate animation speed
    if i < N
        dt = time_steps(i+1) - time_steps(i);
    else
        dt = 1.0;  % Default pause if there's no next step
    end
    pause(dt);  % Adjust speed dynamically based on actual simulation time
end


timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');  % Create a timestamp
filename = ['perfil_obstaculos_' timestamp '.gif'];

for i = 1:N
    set(h, 'YData', data(:, i));  
    title(['Perfil de obstáculos en t = ' num2str(time_steps(i)) ' s']);
    drawnow;
    
    % Capture frame and save to GIF
    frame = getframe(gcf);
    im = frame2im(frame);
    [A, map] = rgb2ind(im, 256);
    
    % Calculate time difference for accurate animation speed
    if i < N
        dt = time_steps(i+1) - time_steps(i);
    else
        dt = 1.0;  % Default pause if there's no next step
    end
    
    if i == 1
        imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', dt);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', dt);
    end
end
