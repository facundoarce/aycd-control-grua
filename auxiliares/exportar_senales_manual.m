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
%   archivo = exportar_senales_manual('sdi')
%       Toma las señales de la última corrida del Simulation Data Inspector.
%       Se usa también automáticamente si no hay variable out o si logsout no
%       se puede leer (p. ej. después de detener la simulación a mano).

modelo = 'sistema_completo';
sup    = [modelo '/Controladores de movimiento/Control supervisor global (Nivel 1)'];
planta = [modelo '/Planta física'];
cmd    = [modelo '/Controladores de movimiento/Comandos de operador'];

% Variable en el .mat | Bloque | Puerto de salida
senales = {
    % Izaje manual (mismas señales que tmp/senales_perfil*.mat)
    'y_l',          planta, 6
    'contacto',     planta, 10
    'TLK_OUT',      sup,    4
    'perfil',       [planta '/Carga suspendida/Perfil de obstáculos/Perfil en x_l'], 1  % perfil bajo la carga
    'm_l',          planta, 11
    'm_l_est',      sup,    3
    % Izaje y traslación manual
    % Comandos de operador (joystick o escenario; agregar_comandos_variantes.m)
    'translation',  cmd,    1  % palanca de carro
    'hoist',        cmd,    2  % palanca de izaje
    'TLK_cmd',      cmd,    3  % botón de twistlocks
    'v_h_ref',      sup,    2
    'm_estimated',  sup,    7  % requiere agregar_salida_m_estimated.m
    'v_t_ref',      sup,    1
    'BRK_t',        sup,    5
    'T_tm',         planta, 13  % torque del motor de carro (agregar_T_tm_sensor_bus.m)
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
    if evalin('base', 'exist(''out'', ''var'')') == 1
        arg = evalin('base', 'out');
    else
        warning('No hay variable out en el workspace base: se usa el Simulation Data Inspector.');
        arg = 'sdi';
    end
end

if ischar(arg) && strcmp(arg, 'registrar')
    load_system(modelo);
    for k = 1:size(senales, 1)
        ph = get_param(senales{k, 2}, 'PortHandles');
        if senales{k, 3} > numel(ph.Outport)
            warning('%s: %s no tiene salida %d (no se registra).', ...
                senales{k, 1}, senales{k, 2}, senales{k, 3});
            continue
        end
        set_param(ph.Outport(senales{k, 3}), 'DataLogging', 'on');
    end
    fprintf('Registro activado en %d señales de %s (modelo sin guardar).\n', ...
        size(senales, 1), modelo);
    archivo = '';
    return
end

% Dataset de señales registradas
if ischar(arg) && strcmp(arg, 'sdi')
    logsout = dataset_sdi();
elseif isa(arg, 'Simulink.SimulationOutput')
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
try
    n = logsout.numElements;
catch err
    % logsout lee las señales del SDI recién al usarse; si la simulación se
    % detuvo a mano esa lectura puede fallar: exportar la corrida del SDI
    warning('No se pudo leer logsout (%s): se usa el Simulation Data Inspector.', err.message);
    logsout = dataset_sdi();
    n = logsout.numElements;
end
bloques = repmat({''}, n, 1);
puertos = nan(n, 1);
for i = 1:n
    el = logsout.getElement(i);
    % Sólo señales de salida de bloques (la corrida del SDI puede incluir
    % también otros elementos, p. ej. Simulink.SimulationData.Parameter)
    if ~isa(el, 'Simulink.SimulationData.Signal') || el.BlockPath.getLength() == 0
        continue
    end
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

function ds = dataset_sdi()
% Señales de la última corrida del Simulation Data Inspector como Dataset
ids = Simulink.sdi.getAllRunIDs;
assert(~isempty(ids), 'No hay corridas en el Simulation Data Inspector.');
try
    ds = Simulink.sdi.exportRun(ids(end));
catch
    ds = export(Simulink.sdi.getRun(ids(end)));
end
end
