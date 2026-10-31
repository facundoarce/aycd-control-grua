function varargout = correr_escenario(scenario_name, varargin)
%CORRER_ESCENARIO Simula sistema_completo con comandos de operador programados.
%   [results, file] = correr_escenario('escenario_despegue_estimacion')
%   [results, file] = correr_escenario(scenario_name, 'animacion', true)
%   correr_escenario(scenario_name, 'parametros', struct('k_v_lim', 5))
%       Sin salidas pedidas, sólo imprime la tabla de verificaciones.
%       'parametros': variables que reemplazan en el workspace base a las de
%       parametros.m (para probar otro valor sin editarlo).
%       Ejecuta parametros.m, carga el escenario tmp/escenarios/<scenario_name>.m,
%       arma las timeseries cmd_translation, cmd_hoist y cmd_TLK, simula con
%       modo_comandos = 2 (variante "Escenario" de "Comandos de operador") y
%       sin sincronizar con el tiempo real, exporta las señales a
%       tmp/<scenario_name>_<fecha>.mat y aplica las verificaciones
%       generales y las propias del escenario. Los resultados se agregan al
%       .mat (variable results) y se escriben en un .txt con el mismo nombre.
%
%   Un escenario es una función sin argumentos que devuelve un struct con:
%       description  texto
%       T_end        [s] tiempo de simulación
%       steps        [t, palanca de carro, palanca de izaje, botón TLK] por fila;
%                    cada valor se mantiene hasta la fila siguiente
%       check        (opcional) @(d) -> struct array con campos name, ok, detail;
%                    d son las señales exportadas (timeseries)
%
%   Requiere el subsistema "Comandos de operador" (agregar_comandos_variantes.m).
%   Al terminar, modo_comandos vuelve a 1 y se restaura la sincronización con
%   el tiempo real (el modelo queda modificado sin guardar: no hace falta guardarlo).

opts = struct('animacion', false, 'parametros', struct());
for k = 1:2:numel(varargin)
    opts.(varargin{k}) = varargin{k + 1};
end

model = 'sistema_completo';
root_dir = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(fullfile(root_dir, 'tmp', 'escenarios'));

%% Parámetros y escenario
evalin('base', 'parametros');
names = fieldnames(opts.parametros);
for k = 1:numel(names)
    assignin('base', names{k}, opts.parametros.(names{k}));
    fprintf('%s = %s (en lugar del valor de parametros.m)\n', names{k}, ...
        mat2str(opts.parametros.(names{k})));
end
scen = feval(scenario_name);
t = scen.steps(:, 1);
assignin('base', 'cmd_translation', timeseries(scen.steps(:, 2), t));
assignin('base', 'cmd_hoist', timeseries(scen.steps(:, 3), t));
assignin('base', 'cmd_TLK', timeseries(logical(scen.steps(:, 4)), t));
assignin('base', 'modo_comandos', 2);
assignin('base', 'animacion_on', opts.animacion);
fprintf('Escenario %s: %s\n', scenario_name, scen.description);

%% Simulación sin sincronizar con el tiempo real
load_system(model);
pacing = get_param(model, 'EnablePacing');
set_param(model, 'EnablePacing', 'off');
cleanup = onCleanup(@() restore_model(model, pacing)); %#ok<NASGU>
exportar_senales_manual('registrar');
% La variante "Escenario" de "Comandos de operador" requiere modo_comandos = 2
cmd_mode = evalin('base', 'modo_comandos');
fprintf('modo_comandos = %d antes de simular\n', cmd_mode);
assert(cmd_mode == 2, 'modo_comandos = %d: se esperaba 2 (variante Escenario)', cmd_mode);
tic;
out = sim(model, 'StopTime', num2str(scen.T_end), 'ReturnWorkspaceOutputs', 'on');
fprintf('Simulación de %.1f s en %.1f s\n', scen.T_end, toc);
assignin('base', 'out', out);

%% Exportación y verificaciones
file = exportar_senales_manual(out, fullfile(root_dir, 'tmp', ...
    [scenario_name '_' datestr(now, 'yyyy-mm-dd_HH-MM-SS') '.mat']));
d = load(file);
results = general_checks(d);
if isfield(scen, 'check')
    results = [results, scen.check(d)];
end
print_results(results);
% Resultados junto a las señales: variable results en el .mat y tabla en un
% .txt con el mismo nombre
save(file, 'results', '-append');
[folder, base] = fileparts(file);
fid = fopen(fullfile(folder, [base '.txt']), 'w', 'n', 'windows-1252');
fprintf(fid, 'Escenario %s: %s\n', scenario_name, scen.description);
print_results(results, fid);
fclose(fid);
varargout = {results, file};
varargout = varargout(1:nargout);
end

%% Funciones locales

function restore_model(model, pacing)
set_param(model, 'EnablePacing', pacing);
assignin('base', 'modo_comandos', 1);
end

function results = general_checks(d)
% Límites de la guía: estimación de masa 1 % (3.7), sobrevelocidad 115 % (3.2),
% límites de emergencia del carro y del izaje (3.2, 3.3), balanceo 20° (4.2); y la carga nunca
% por encima del carro (detecta colisiones que la planta resuelve lanzando la carga)
p = @(n) base_value(n);
dt = 0.02;
tg = (0:dt:d.y_l.Time(end))';
y_l = sample_ts(d.y_l, tg);
% Altura de izaje y_h = Y_t0 - l_h (tambor: la miden los fines de carrera de
% izaje); con balanceo, y_l sube por el arco del péndulo. Si no se exportó
% l_h, se usa y_l
if isfield(d, 'l_h')
    y_h = p('Y_t0') - sample_ts(d.l_h, tg);
    y_name = 'y_h';
else
    y_h = y_l;
    y_name = 'y_l';
end
results = struct('name', {}, 'ok', {}, 'detail', {});

% Estimación de masa: error en cada nueva estimación
t_est = d.m_l_est.Time(find(diff(double(d.m_l_est.Data(:))) ~= 0) + 1);
if isempty(t_est)
    results(end + 1) = check_row('Estimación de masa < 1 %', NaN, 'no hubo estimación');
else
    m_est = sample_ts(d.m_l_est, t_est);
    m_l = sample_ts(d.m_l, t_est);
    err = abs(m_est - m_l) ./ m_l * 100;
    results(end + 1) = check_row('Estimación de masa < 1 %', all(err < 1), ...
        sprintf('%s %% en t = %s s', mat2str(err', 2), mat2str(t_est', 4)));
end

v_y = abs(diff(y_h)) / dt;
results(end + 1) = check_row('Velocidad de izaje <= 115 % v_h_max', max(v_y) <= 1.15 * p('v_h_max'), ...
    sprintf('máx. %.2f m/s (límite %.2f)', max(v_y), 1.15 * p('v_h_max')));
v_t = max(abs(d.x_t_dot.Data(:)));
results(end + 1) = check_row('Velocidad de carro <= 115 % v_t_max', v_t <= 1.15 * p('v_t_max'), ...
    sprintf('máx. %.2f m/s (límite %.2f)', v_t, 1.15 * p('v_t_max')));
x_t = d.x_t.Data(:);
results(end + 1) = check_row('Carro sin llegar a los límites de emergencia', ...
    min(x_t) > p('x_t_min_emer') && max(x_t) < p('x_t_max_emer'), ...
    sprintf('x_t entre %.2f y %.2f m (emergencia %.1f / %.1f)', min(x_t), max(x_t), ...
    p('x_t_min_emer'), p('x_t_max_emer')));
results(end + 1) = check_row('Izaje sin llegar a los límites de emergencia', ...
    min(y_h) > p('y_h_min_emer') && max(y_h) < p('y_h_max_emer'), ...
    sprintf('%s entre %.2f y %.2f m (emergencia %.1f / %.1f)', y_name, min(y_h), max(y_h), ...
    p('y_h_min_emer'), p('y_h_max_emer')));
th = max(abs(d.theta_l.Data(:))) * 180 / pi;
results(end + 1) = check_row('Balanceo <= 20°', th <= 20, sprintf('máx. %.1f°', th));
results(end + 1) = check_row('Carga por debajo del carro', max(y_l) < p('Y_t0'), ...
    sprintf('y_l máx. %.2f m (Y_t0 = %.1f)', max(y_l), p('Y_t0')));
end

function r = check_row(name, ok, detail)
r = struct('name', name, 'ok', ok, 'detail', detail);
end

function print_results(results, fid)
% fid: 1 (consola, por defecto) o un archivo abierto con fopen
if nargin < 2
    fid = 1;
end
fprintf(fid, '\n%-45s %-8s %s\n', 'Verificación', 'Result.', 'Detalle');
for k = 1:numel(results)
    if isnan(results(k).ok)
        r = 'N/A';
    elseif results(k).ok
        r = 'PASA';
    else
        r = 'FALLA';
    end
    fprintf(fid, '%-45s %-8s %s\n', results(k).name, r, results(k).detail);
end
end

function v = base_value(var_name)
% Valor de una variable del workspace base (Simulink.Parameter o numérica)
v = evalin('base', var_name);
if isa(v, 'Simulink.Parameter')
    v = v.Value;
end
end

function v = sample_ts(ts, t)
% Muestras de un timeseries escalar en los instantes t (último valor anterior)
[t_unique, idx] = unique(ts.Time(:), 'last');
data = double(ts.Data(:));
v = interp1(t_unique, data(idx), t(:), 'previous', 'extrap');
end
