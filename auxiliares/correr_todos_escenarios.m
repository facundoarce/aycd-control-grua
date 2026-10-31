function summary = correr_todos_escenarios(varargin)
%CORRER_TODOS_ESCENARIOS Corre todos los escenarios de tmp/escenarios.
%   correr_todos_escenarios
%   correr_todos_escenarios('parametros', struct('k_v_lim', 5))
%       Corre uno detrás del otro, en orden alfabético, todos los escenarios
%       tmp/escenarios/*.m con correr_escenario (mismas opciones). Cada
%       escenario guarda sus señales y resultados en tmp/<escenario>_<fecha>.mat
%       y .txt. Si un escenario da error, se informa y se sigue con el resto.
%       Al final imprime un resumen y lo guarda en
%       tmp/resumen_escenarios_<fecha>.txt.
%   summary = correr_todos_escenarios(...)
%       Devuelve el resumen (struct array con name, passed, total, file, error).

root_dir = fullfile(fileparts(mfilename('fullpath')), '..');
files = dir(fullfile(root_dir, 'tmp', 'escenarios', '*.m'));
names = sort(cellfun(@(f) f(1:end - 2), {files.name}, 'UniformOutput', false));
assert(~isempty(names), 'No hay escenarios en tmp/escenarios.');

summary = struct('name', names, 'passed', NaN, 'total', NaN, 'file', '', 'error', '');
t_start = tic;
for k = 1:numel(names)
    fprintf('\n===== [%d/%d] %s =====\n', k, numel(names), names{k});
    try
        [results, file] = correr_escenario(names{k}, varargin{:});
        ok = [results.ok];
        summary(k).passed = sum(ok == 1);
        summary(k).total = sum(~isnan(ok));
        summary(k).file = file;
    catch err
        summary(k).error = err.message;
        fprintf(2, 'Error en %s: %s\n', names{k}, err.message);
    end
end

stamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
fprintf('\nResumen (%d escenarios en %.0f s)\n', numel(names), toc(t_start));
print_summary(summary, 1);
fid = fopen(fullfile(root_dir, 'tmp', ['resumen_escenarios_' stamp '.txt']), 'w', 'n', 'windows-1252');
print_summary(summary, fid);
fclose(fid);
if nargout == 0
    clear summary
end
end

function print_summary(summary, fid)
fprintf(fid, '%-35s %-10s %s\n', 'Escenario', 'Resultado', 'Archivo / error');
for k = 1:numel(summary)
    s = summary(k);
    if ~isempty(s.error)
        fprintf(fid, '%-35s %-10s %s\n', s.name, 'ERROR', s.error);
    else
        if s.passed == s.total
            r = 'PASA';
        else
            r = 'FALLA';
        end
        [~, base] = fileparts(s.file);
        fprintf(fid, '%-35s %-10s %d/%d  %s\n', s.name, r, s.passed, s.total, base);
    end
end
end
