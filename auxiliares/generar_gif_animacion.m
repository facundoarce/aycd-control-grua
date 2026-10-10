function gif = generar_gif_animacion(archivo, dt, velocidad)
%GENERAR_GIF_ANIMACION Genera un GIF de la animación de la grúa desde una corrida exportada.
%   gif = generar_gif_animacion
%   gif = generar_gif_animacion(archivo)
%   gif = generar_gif_animacion(archivo, dt)
%   gif = generar_gif_animacion(archivo, dt, velocidad)
%       Reproduce con animar_grua.m las señales guardadas por
%       exportar_senales_manual.m y escribe un cuadro cada dt segundos de
%       simulación (por defecto 0.1 s). Cada cuadro dura dt/velocidad en el
%       GIF (por defecto velocidad = 1: velocidad real). Sin archivo, usa el
%       tmp/senales_manual_*.mat más reciente. El GIF se guarda junto al .mat
%       con sufijo _animacion.gif.
%   La generación no espera entre cuadros: tarda lo que lleva dibujar y
%   capturar cada uno. Para acelerarla, la figura se lleva a un tamaño fijo
%   (tam_gif) y la paleta de colores se calcula una sola vez.
%   Requiere parametros.m ejecutado (geometría de la grúa y de la grilla).

tam_gif = [1280 720];  % [px] ancho y alto de la figura al capturar

dir_tmp = fullfile(fileparts(mfilename('fullpath')), '..', 'tmp');
if nargin < 1 || isempty(archivo)
    lista = dir(fullfile(dir_tmp, 'senales_manual_*.mat'));
    assert(~isempty(lista), 'No hay archivos senales_manual_*.mat en %s', dir_tmp);
    [~, i] = max([lista.datenum]);
    archivo = fullfile(lista(i).folder, lista(i).name);
end
if nargin < 2 || isempty(dt)
    dt = 0.1;  % [s]
end
if nargin < 3 || isempty(velocidad)
    velocidad = 1;
end
delay = dt / velocidad;  % [s] duración de cada cuadro en el GIF
if delay < 0.02
    % Muchos visores muestran los cuadros de menos de 0.02 s a 0.1 s
    warning('Duración de cuadro %.3f s < 0.02 s: aumentar dt en lugar de velocidad.', delay);
end

d = load(archivo);
necesarias = {'x_t', 'x_t_dot', 'x_l', 'y_l', 'theta_l', 'TLK_OUT', 'contacto'};
faltan = necesarias(~isfield(d, necesarias));
assert(isempty(faltan), 'Faltan señales en %s: %s', archivo, strjoin(faltan, ', '));

% Grilla de tiempo de los cuadros, dentro del rango común de todas las señales
t_fin = min(cellfun(@(n) d.(n).Time(end), necesarias));
t = 0:dt:t_fin;

% Valor de cada señal en los instantes de los cuadros (último valor anterior)
x_t     = muestrear(d.x_t, t);
x_t_dot = muestrear(d.x_t_dot, t);
x_l     = muestrear(d.x_l, t);
y_l     = muestrear(d.y_l, t);
theta_l = muestrear(d.theta_l, t);
TLK     = muestrear(d.TLK_OUT, t) > 0.5;
cont    = muestrear(d.contacto, t) > 0.5;

% Perfil de obstáculos: vector por slot (exportado desde 2026-10-10)
if isfield(d, 'y_c0')
    y_c0 = muestrear(d.y_c0, t);  % [n_t x N_slots]
else
    warning(['%s no tiene y_c0: se usa el perfil inicial y_c0_0 del workspace ' ...
        '(los contenedores no cambian con TLK).'], archivo);
    y0 = evalin('base', 'y_c0_0');
    if isa(y0, 'Simulink.Parameter')
        y0 = y0.Value;
    end
    y_c0 = repmat(y0(:).', numel(t), 1);
end

[carpeta, nombre] = fileparts(archivo);
gif = fullfile(carpeta, [nombre '_animacion.gif']);

tic;
for k = 1:numel(t)
    fig = animar_grua(t(k), x_t(k), x_t_dot(k), x_l(k), y_l(k), theta_l(k), ...
        TLK(k), y_c0(k, :), cont(k), true);
    if k == 1
        % Tamaño fijo (la figura se crea maximizada) y esperar a que la
        % ventana lo tome antes de fijar el tamaño del GIF con el primer cuadro
        set(fig, 'WindowState', 'normal', 'Units', 'pixels');
        set(fig, 'Position', [50 50 tam_gif]);
        drawnow;
        pause(0.5);
    end
    drawnow;
    img = frame2im(getframe(fig));
    if k == 1
        tam = [size(img, 1) size(img, 2)];
        % Paleta única para todo el GIF, a partir del primer cuadro más una
        % franja con los colores del contenedor enganchado (en animar_grua),
        % que no aparece en el primer cuadro si TLK está abierto
        franja = repmat(uint8(255 * reshape([0.15 0.55 0.85; 0.05 0.25 0.45], 1, 2, 3)), ...
            8, ceil(tam(2) / 2));
        [~, map] = rgb2ind([img; franja(:, 1:tam(2), :)], 256);
    elseif ~isequal([size(img, 1) size(img, 2)], tam)
        % El GIF toma el tamaño del primer cuadro: los cuadros de otro tamaño
        % se verían recortados
        img = redimensionar(img, tam);
    end
    A = rgb2ind(img, map, 'nodither');
    if k == 1
        imwrite(A, map, gif, 'gif', 'LoopCount', Inf, 'DelayTime', delay);
    else
        imwrite(A, map, gif, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
    end
end
fprintf('%d cuadros guardados en %s (%.1f s)\n', numel(t), gif, toc);
end

function img = redimensionar(img, tam)
% Remuestreo por vecino más cercano a tam = [filas columnas] (sin toolboxes)
filas = round(linspace(1, size(img, 1), tam(1)));
columnas = round(linspace(1, size(img, 2), tam(2)));
img = img(filas, columnas, :);
end

function v = muestrear(ts, t)
% Muestras de un timeseries en los instantes t, manteniendo el último valor
% anterior (zoh). Devuelve una fila por instante y una columna por elemento.
datos = double(ts.Data);
n_t = numel(ts.Time);
if ndims(datos) == 3                % señal vectorial registrada como N x 1 x n_t
    datos = reshape(datos, [], n_t).';
elseif size(datos, 1) ~= n_t        % señal vectorial registrada como N x n_t
    datos = datos.';
end
% Instantes repetidos (pasos menores del solver): quedarse con el último
[tiempo, idx] = unique(ts.Time(:), 'last');
v = interp1(tiempo, datos(idx, :), t(:), 'previous', 'extrap');
end
