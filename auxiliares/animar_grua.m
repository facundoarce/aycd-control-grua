function fig = animar_grua(t, x_t, x_t_dot, x_l, y_l, theta_l, TLK, y_c0, contacto, m_estimated, ignorar_on)
%ANIMAR_GRUA Dibuja la grúa, la carga y el perfil de obstáculos durante la simulación.
%   Se llama como función extrínseca desde el bloque MATLAB Function
%   "Animación" de sistema_completo (agregado con agregar_animacion.m).
%   Entradas: tiempo de simulación, posición y velocidad del carro, posición
%   de la carga (base del spreader), ángulo de balanceo, estado de twistlocks,
%   vector de perfil de obstáculos (un valor por slot), contacto de la carga
%   con el perfil (apoyada) y masa de la carga estimada por el supervisor
%   (NaN si no se conoce, p. ej. en corridas exportadas sin esa señal).
%
%   Los parámetros geométricos se leen del workspace base (parametros.m).
%   Para desactivar la animación sin tocar el modelo: animacion_on = false
%   en el workspace base.
%   generar_gif_animacion.m la usa para reproducir una corrida exportada:
%   con ignorar_on = true dibuja aunque animacion_on sea false, y devuelve el
%   handle de la figura para capturar los cuadros.
%
%   El carro y el spreader se dibujan en forma esquemática: sus dimensiones
%   sólo son de visualización (no son parámetros del modelo).

persistent S

fig = [];
if nargin < 11
    ignorar_on = false;
end
if ~ignorar_on && evalin('base', 'exist(''animacion_on'', ''var'') && ~animacion_on')
    return
end

% Inicializar al empezar una simulación nueva o si se cerró la figura
if isempty(S) || ~isvalid(S.fig) || t < S.t_prev
    S = crear_figura(y_c0);
end
S.t_prev = t;
fig = S.fig;

% Carro, cable y spreader
set(S.carro, 'XData', x_t + S.carro_x);
set(S.cable, 'XData', [x_t x_l], 'YData', [S.Y_t0 y_l]);
set(S.spreader, 'XData', x_l + [-1 1] * S.W_c / 2, 'YData', [y_l y_l]);

% Contenedor enganchado (twistlocks cerrados): debajo del spreader
if TLK
    set(S.carga, 'XData', x_l + [-1 1 1 -1] * S.W_c / 2, ...
        'YData', y_l - [S.H_c S.H_c 0 0], 'Visible', 'on');
else
    set(S.carga, 'Visible', 'off');
end

% Perfil de obstáculos: redibujar sólo si cambió (flancos de TLK)
if ~isequal(y_c0(:), S.y_c0)
    S.y_c0 = y_c0(:);
    actualizar_perfil(S);
end

set(S.texto, 'String', sprintf(['t = %6.1f s\n' ...
    'x_t = %6.2f m   v_t = %5.2f m/s\n' ...
    'x_l = %6.2f m   y_l = %6.2f m\n' ...
    '\\theta_l = %5.1f°   TLK = %d   contacto = %d\n' ...
    'masa: %s'], ...
    t, x_t, x_t_dot, x_l, y_l, theta_l * 180 / pi, TLK, contacto, texto_masa(m_estimated)));

drawnow limitrate;
end

%% Funciones locales

function S = crear_figura(y_c0)
% Crea la figura con los objetos fijos y devuelve los handles de los móviles
p = @(nombre) valor_base(nombre);
S.W_c = p('W_c');
S.H_c = p('H_c');
S.Y_t0 = p('Y_t0');
N = p('N_slots');
x_slot0 = p('x_slot0');
y_quay = p('y_c0_quay');
y_ship = p('y_c0_ship');
x_min_oper = p('x_t_min_oper');
x_max_oper = p('x_t_max_oper');

S.x_edges = x_slot0 + (0:N) * S.W_c;
x_centros = S.x_edges(1:end-1) + S.W_c / 2;
S.y_base = y_ship * ones(N, 1);
S.y_base(x_centros < 0) = y_quay;
S.y_c0 = nan(N, 1);
S.t_prev = -Inf;

y_min = y_ship - 2;
y_max = S.Y_t0 + 4;
x_lim = [S.x_edges(1) S.x_edges(end)] + [-5 5];

S.fig = figure('Name', 'Animación grúa', 'NumberTitle', 'off', 'Color', 'w', ...
    'WindowState', 'maximized');
S.ax = axes(S.fig);
hold(S.ax, 'on');

% Cuadrícula de fondo W_c x H_c referida a la base de cada lado
color_grilla = [0.90 0.90 0.90];
plot(S.ax, cuadricula_x(S.x_edges), cuadricula_y(S.x_edges, y_min, S.Y_t0), ...
    'Color', color_grilla, 'HandleVisibility', 'off');
[xg, yg] = filas_grilla(S.x_edges(1), 0, y_quay, S.H_c, S.Y_t0);
plot(S.ax, xg, yg, 'Color', color_grilla, 'HandleVisibility', 'off');
[xg, yg] = filas_grilla(0, S.x_edges(end), y_ship, S.H_c, S.Y_t0);
plot(S.ax, xg, yg, 'Color', color_grilla, 'HandleVisibility', 'off');

% Bases de muelle y barco, borde del muelle
plot(S.ax, [x_lim(1) 0], [y_quay y_quay], 'Color', [0.35 0.35 0.35], ...
    'LineWidth', 4, 'DisplayName', 'Base muelle');
plot(S.ax, [0 x_lim(2)], [y_ship y_ship], 'Color', [0.00 0.30 0.65], ...
    'LineWidth', 4, 'DisplayName', 'Base barco');
plot(S.ax, [0 0], [y_min y_quay], 'Color', [0.35 0.35 0.35], 'LineWidth', 2, ...
    'HandleVisibility', 'off');

% Viga del carro y límites de operación de traslación
plot(S.ax, x_lim, [S.Y_t0 S.Y_t0], 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
plot(S.ax, [x_min_oper x_min_oper; x_max_oper x_max_oper]', [y_min y_max; y_min y_max]', ...
    'r:', 'LineWidth', 1, 'HandleVisibility', 'off');

% Contenedores apilados y perfil de obstáculos
S.cont = patch(S.ax, 'Vertices', zeros(0, 2), 'Faces', zeros(0, 4), ...
    'FaceColor', [0.95 0.65 0.30], 'EdgeColor', [0.55 0.30 0.10], ...
    'DisplayName', 'Contenedores');
S.perfil = stairs(S.ax, S.x_edges, nan(1, N + 1), 'Color', [0.80 0.10 0.10], ...
    'LineWidth', 1.5, 'DisplayName', 'Perfil de obstáculos');

% Objetos móviles: carro (esquemático), cable, spreader y contenedor enganchado
S.carro_x = [-1 1 1 -1] * 2;
S.carro = patch(S.ax, S.carro_x, S.Y_t0 + [0 0 1.5 1.5], [0.20 0.20 0.20], ...
    'DisplayName', 'Carro');
S.cable = plot(S.ax, [0 0], [S.Y_t0 S.Y_t0], 'k-', 'LineWidth', 1, ...
    'HandleVisibility', 'off');
S.spreader = plot(S.ax, [0 0], [0 0], 'Color', [0.95 0.80 0.10], 'LineWidth', 4, ...
    'DisplayName', 'Spreader');
S.carga = patch(S.ax, zeros(1, 4), zeros(1, 4), [0.15 0.55 0.85], ...
    'EdgeColor', [0.05 0.25 0.45], 'Visible', 'off', 'HandleVisibility', 'off');

S.texto = text(S.ax, x_lim(1) + 1, y_max - 1, '', 'VerticalAlignment', 'top', ...
    'FontName', 'FixedWidth', 'BackgroundColor', 'w', 'EdgeColor', [0.7 0.7 0.7]);

hold(S.ax, 'off');
box(S.ax, 'on');
axis(S.ax, 'equal');
xlim(S.ax, x_lim);
ylim(S.ax, [y_min y_max]);
xlabel(S.ax, 'x [m]');
ylabel(S.ax, 'y [m]');
legend(S.ax, 'Location', 'northeast');

S.y_c0 = y_c0(:);
actualizar_perfil(S);
end

function s = texto_masa(m_estimated)
% Estado de la estimación de masa para el recuadro de texto
if isnan(double(m_estimated))
    s = 's/d';
elseif m_estimated
    s = 'estimada';
else
    s = 'estimando (traslación bloqueada)';
end
end

function actualizar_perfil(S)
[V, F] = contenedores(S.y_c0, S.y_base, S.x_edges, S.H_c);
set(S.cont, 'Vertices', V, 'Faces', F);
set(S.perfil, 'YData', [S.y_c0; S.y_c0(end)]);
end

function v = valor_base(nombre)
% Valor de una variable del workspace base (Simulink.Parameter o numérica)
v = evalin('base', nombre);
if isa(v, 'Simulink.Parameter')
    v = v.Value;
end
end

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
