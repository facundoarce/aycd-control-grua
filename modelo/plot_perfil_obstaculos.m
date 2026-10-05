% PLOT_PERFIL_OBSTACULOS Grafica y anima la evolución del perfil de obstáculos por slots.
%
% Requiere:
%   - parametros.m ejecutado (x_slot0, N_slots, W_c, H_c, y_c0_0, y_c0_quay, y_c0_ship).
%   - Salida de la simulación en "out" con el bloque To Workspace "y_c0"
%     (formato Structure With Time). Como el subsistema "Perfil de obstáculos"
%     se ejecuta sólo en los flancos de TLK, hay una muestra por flanco.
%
% Genera una figura animada y un GIF con el perfil en cada flanco de TLK.
% Cada contenedor se dibuja como un rectángulo sobre una cuadrícula de fondo
% de W_c x H_c referida a la base de cada lado (muelle y barco).

%% Datos
% Perfil inicial (t = 0) más el perfil registrado en cada flanco de TLK
N = N_slots.Value;
t_log = out.y_c0.time(:);
v = out.y_c0.signals.values;
if ndims(v) == 3          % señal [1 x N] registrada como 1 x N x n_t
    y_log = reshape(v, N, []);
elseif size(v, 2) == N    % señal registrada como n_t x N
    y_log = v.';
else                      % una sola muestra registrada como N x 1
    y_log = v(:);
end
time_steps = [0; t_log];
data = [y_c0_0.Value(:), y_log];  % [N_slots x n_t] un perfil por columna
n_t = numel(time_steps);

% Bordes de los slots (grilla alineada con x = 0) para graficar en escalones
x_edges = x_slot0.Value + (0:N) * W_c.Value;
perfil_escalones = @(y) [y(:); y(end)];  % stairs necesita N+1 valores para N+1 bordes

% Altura base de cada slot (misma clasificación por centro que parametros.m)
x_centros = x_edges(1:end-1) + W_c.Value/2;
y_base = y_c0_ship * ones(N, 1);
y_base(x_centros < 0) = y_c0_quay;

% Límites fijos de los ejes para toda la animación
y_min = min([data(:); y_c0_ship]) - H_c.Value;
y_max = max([data(:); y_c0_quay]) + H_c.Value;

%% Figura
figure;
hold on;

% Cuadrícula de fondo: una celda por posición de contenedor (W_c x H_c),
% con las filas referidas a la base de cada lado
color_grilla = [0.88 0.88 0.88];
plot(cuadricula_x(x_edges), cuadricula_y(x_edges, y_min, y_max), ...
    'Color', color_grilla, 'HandleVisibility', 'off');
[xg, yg] = filas_grilla(x_edges(1), 0, y_c0_quay, H_c.Value, y_max);
plot(xg, yg, 'Color', color_grilla, 'HandleVisibility', 'off');
[xg, yg] = filas_grilla(0, x_edges(end), y_c0_ship, H_c.Value, y_max);
plot(xg, yg, 'Color', color_grilla, 'HandleVisibility', 'off');

% Contenedores apilados en cada slot
[V, F] = contenedores(data(:, 1), y_base, x_edges, H_c.Value);
h_cont = patch('Vertices', V, 'Faces', F, 'FaceColor', [0.95 0.65 0.30], ...
    'EdgeColor', [0.55 0.30 0.10], 'DisplayName', 'Contenedores');

% Bases de muelle y barco
plot([x_edges(1) 0], [y_c0_quay y_c0_quay], 'Color', [0.35 0.35 0.35], ...
    'LineWidth', 4, 'DisplayName', 'Base muelle');
plot([0 x_edges(end)], [y_c0_ship y_c0_ship], 'Color', [0.00 0.30 0.65], ...
    'LineWidth', 4, 'DisplayName', 'Base barco');
plot([0 0], [y_min y_max], 'k--', 'HandleVisibility', 'off');  % borde muelle/barco

% Perfil de obstáculos
h = stairs(x_edges, perfil_escalones(data(:, 1)), 'Color', [0.80 0.10 0.10], ...
    'LineWidth', 2, 'DisplayName', 'Perfil de obstáculos');

hold off;
box on;
xlim([x_edges(1) x_edges(end)]);
ylim([y_min y_max]);
xlabel('x [m]');
ylabel('y_{c0} [m]');
legend('Location', 'northeast');

%% Animación y exportación a GIF
dt_frame = 1.0;  % [s] duración de cada cuadro (los flancos de TLK no son equiespaciados)
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
filename = ['perfil_obstaculos_' timestamp '.gif'];

for i = 1:n_t
    set(h, 'YData', perfil_escalones(data(:, i)));
    [V, F] = contenedores(data(:, i), y_base, x_edges, H_c.Value);
    set(h_cont, 'Vertices', V, 'Faces', F);
    title(['Perfil de obstáculos en t = ' num2str(time_steps(i), '%.2f') ' s']);
    drawnow;

    % Capturar cuadro y agregarlo al GIF
    frame = getframe(gcf);
    [A, map] = rgb2ind(frame2im(frame), 256);
    if i == 1
        imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', dt_frame);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', dt_frame);
    end

    pause(dt_frame);
end

%% Funciones locales

function [V, F] = contenedores(y, y_base, x_edges, H_c)
% Vértices y caras de un rectángulo por contenedor apilado en cada slot
V = zeros(0, 2);
F = zeros(0, 4);
for k = 1:numel(y)
    n_stack = max(0, round((y(k) - y_base(k)) / H_c));  % contenedores en el slot k
    for j = 1:n_stack
        y0 = y_base(k) + (j-1) * H_c;
        V = [V; x_edges(k) y0; x_edges(k+1) y0; x_edges(k+1) y0+H_c; x_edges(k) y0+H_c]; %#ok<AGROW>
        F = [F; size(V, 1) + (-3:0)]; %#ok<AGROW>
    end
end
end

function x = cuadricula_x(x_edges)
% Líneas verticales en los bordes de los slots, separadas por NaN
x = reshape([x_edges; x_edges; nan(size(x_edges))], 1, []);
end

function y = cuadricula_y(x_edges, y_min, y_max)
n = numel(x_edges);
y = reshape([y_min*ones(1, n); y_max*ones(1, n); nan(1, n)], 1, []);
end

function [x, y] = filas_grilla(x_ini, x_fin, y_base, H_c, y_max)
% Líneas horizontales cada H_c desde la base hasta y_max, separadas por NaN
niveles = y_base:H_c:y_max;
n = numel(niveles);
x = reshape([x_ini*ones(1, n); x_fin*ones(1, n); nan(1, n)], 1, []);
y = reshape([niveles; niveles; nan(1, n)], 1, []);
end
