function archivo = exportar_senales_manual(arg, archivo)
%EXPORTAR_SENALES_MANUAL Exporta las señales para analizar izaje y traslación manual.
%   exportar_senales_manual('registrar')
%       Activa el registro (DataLogging) de todas las señales de la tabla en
%       sistema_completo. No guarda el modelo.
%   archivo = exportar_senales_manual
%   archivo = exportar_senales_manual(out)
%   archivo = exportar_senales_manual(out, archivo)
%       Toma las señales de out.logsout (o de un Simulink.SimulationData.Dataset)
%       identificándolas por bloque y puerto (no por nombre, para evitar
%       señales duplicadas como M_cx) y las guarda como timeseries en un .mat.
%       Por defecto: tmp/senales_manual_<fecha>.mat.
%       Sin argumentos (o con out vacío) usa la variable out del workspace base.

modelo = 'sistema_completo';
sup    = [modelo '/Controladores de movimiento/Control supervisor global (Nivel 1)'];
planta = [modelo '/Planta física'];

% Variable en el .mat | Bloque | Puerto de salida
senales = {
    % Izaje manual (mismas señales que tmp/senales_perfil*.mat)
    'y_l',          planta, 6
    'contacto',     planta, 10
    'TLK_OUT',      sup,    4
    'perfil',       [planta '/Carga suspendida/Perfil de obstáculos'], 1
    'm_l',          planta, 11
    'm_l_est',      sup,    3
    % Traslación manual
    'translation',  [modelo '/Controladores de movimiento/Selector'], 1
    'v_t_ref',      sup,    1
    'BRK_t',        sup,    5
    'x_t',          planta, 1
    'x_t_dot',      planta, 2
    'x_l',          planta, 5
    'theta_l',      planta, 8
    'F_hw',         planta, 7
    % Animación (generar_gif_animacion.m)
    'y_c0',         planta, 12
    };

% Por defecto: variable out del workspace base (la que deja la simulación)
if nargin < 1 || isempty(arg)
    assert(evalin('base', 'exist(''out'', ''var'')') == 1, ...
        'No hay variable out en el workspace base: simular primero.');
    arg = evalin('base', 'out');
end

if ischar(arg) && strcmp(arg, 'registrar')
    load_system(modelo);
    for k = 1:size(senales, 1)
        ph = get_param(senales{k, 2}, 'PortHandles');
        set_param(ph.Outport(senales{k, 3}), 'DataLogging', 'on');
    end
    fprintf('Registro activado en %d señales de %s (modelo sin guardar).\n', ...
        size(senales, 1), modelo);
    archivo = '';
    return
end

% Dataset de señales registradas
if isa(arg, 'Simulink.SimulationOutput')
    logsout = arg.get('logsout');
else
    logsout = arg;
end
assert(isa(logsout, 'Simulink.SimulationData.Dataset'), ...
    'Se esperaba out (Simulink.SimulationOutput) o logsout (Dataset).');

if nargin < 2 || isempty(archivo)
    dir_tmp = fullfile(fileparts(mfilename('fullpath')), '..', 'tmp');
    if ~exist(dir_tmp, 'dir')
        mkdir(dir_tmp);
    end
    archivo = fullfile(dir_tmp, ['senales_manual_' datestr(now, 'yyyy-mm-dd_HH-MM-SS') '.mat']);
end

% Bloque y puerto de cada elemento registrado
n = logsout.numElements;
bloques = cell(n, 1);
puertos = zeros(n, 1);
for i = 1:n
    el = logsout.getElement(i);
    bloques{i} = el.BlockPath.getBlock(el.BlockPath.getLength());
    puertos(i) = el.PortIndex;
end

datos = struct();
faltantes = {};
for k = 1:size(senales, 1)
    idx = find(strcmp(bloques, senales{k, 2}) & puertos == senales{k, 3});
    if isempty(idx)
        faltantes{end + 1} = senales{k, 1}; %#ok<AGROW>
        continue
    end
    if numel(idx) > 1
        warning('%s: %d elementos registrados, se usa el primero.', senales{k, 1}, numel(idx));
    end
    datos.(senales{k, 1}) = logsout.getElement(idx(1)).Values;
end

if ~isempty(faltantes)
    warning(['Señales no registradas: %s.\nEjecutar exportar_senales_manual(''registrar'') ' ...
        'y volver a simular.'], strjoin(faltantes, ', '));
end
assert(~isempty(fieldnames(datos)), 'Ninguna señal de la tabla está en logsout.');

save(archivo, '-struct', 'datos');
fprintf('%d señales guardadas en %s\n', numel(fieldnames(datos)), archivo);
end
