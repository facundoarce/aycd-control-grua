function exportar_diagramas(dir_salida, modelo)
%EXPORTAR_DIAGRAMAS Exporta los diagramas del modelo como imágenes vectoriales.
%   exportar_diagramas
%   exportar_diagramas(dir_salida)
%   exportar_diagramas(dir_salida, modelo)
%       Exporta el nivel superior y cada subsistema de modelo (por defecto
%       sistema_completo) en formato vectorial (texto y líneas vectoriales, sin
%       pérdida de resolución al hacer zoom). Prueba los formatos de FORMATOS
%       en orden y usa el primero que acepte la versión de Simulink (R2019a no
%       acepta 'epsc'). Los charts de Stateflow se exportan con sfprint si
%       print -s falla.
%       Por defecto dir_salida es figuras/simulink/originales del informe, que
%       contiene a este repositorio como submódulo en ref/aycd-control-grua.
%       No se exportan los bloques MATLAB Function (su código va en los
%       anexos) ni los bloques enmascarados (Signal Builder).
%       Escribe además indice.txt con la correspondencia archivo - bloque.
%       No modifica ni guarda el modelo.
%       En el informe, convertir a PDF recortado con figuras/scripts/recortar_diagramas.sh.

if nargin < 2
    modelo = 'sistema_completo';
end
if nargin < 1 || isempty(dir_salida)
    raiz = fileparts(fileparts(mfilename('fullpath')));
    dir_salida = fullfile(raiz, '..', '..', 'figuras', 'simulink', 'originales');
end
if ~exist(dir_salida, 'dir')
    mkdir(dir_salida);
end

estaba_cargado = bdIsLoaded(modelo);
load_system(modelo);

% Incluye los subsistemas de variantes inactivas (Comandos de operador)
sistemas = [{modelo}; find_system(modelo, 'LookUnderMasks', 'none', ...
    'FollowLinks', 'on', 'Variants', 'AllVariants', 'BlockType', 'SubSystem')];

fid = fopen(fullfile(dir_salida, 'indice.txt'), 'w', 'n', 'UTF-8');
n_ok = 0;
for k = 1:numel(sistemas)
    s = sistemas{k};
    es_bloque = ~strcmp(s, modelo);
    sf_tipo = '';
    if es_bloque
        sf_tipo = get_param(s, 'SFBlockType');
        if strcmp(sf_tipo, 'MATLAB Function') || strcmp(get_param(s, 'Mask'), 'on')
            continue
        end
    end
    base = fullfile(dir_salida, nombre_archivo(s));
    open_system(s);
    [ext, msg] = imprimir(s, base, strcmp(sf_tipo, 'Chart'));
    if es_bloque
        close_system(s);
    end
    if isempty(ext)
        warning('No se pudo exportar %s: %s', strrep(s, newline, ' '), msg);
        continue
    end
    fprintf(fid, '%s%s\t%s\n', nombre_archivo(s), ext, strrep(s, newline, ' '));
    fprintf('Exportado (%s): %s\n', ext, strrep(s, newline, ' '));
    n_ok = n_ok + 1;
end
fclose(fid);

if ~estaba_cargado
    close_system(modelo, 0);
end
fprintf('%d diagramas exportados en %s\n', n_ok, dir_salida);
end

function [ext, msg] = imprimir(s, base, es_chart)
% Imprime s en el primer formato vectorial aceptado. Devuelve la extensión
% del archivo escrito, o vacío y el último mensaje de error.
% Formato de print/sfprint | extensión
FORMATOS = {'pdf', '.pdf'; 'svg', '.svg'; 'eps', '.eps'; 'psc', '.ps'};
ext = '';
msg = '';
for k = 1:size(FORMATOS, 1)
    archivo = [base FORMATOS{k, 2}];
    try
        print(['-s' s], ['-d' FORMATOS{k, 1}], archivo);
    catch err
        msg = err.message;
        if ~es_chart
            continue
        end
        try
            sfprint(s, FORMATOS{k, 1}, archivo);
        catch err
            msg = err.message;
            continue
        end
    end
    ext = FORMATOS{k, 2};
    return
end
end

function nombre = nombre_archivo(ruta)
% Ruta del bloque -> nombre de archivo ASCII: niveles separados por '-'
nombre = strrep(ruta, newline, ' ');
nombre = regexprep(nombre, {'[áÁ]', '[éÉ]', '[íÍ]', '[óÓ]', '[úÚ]', '[ñÑ]'}, ...
    {'a', 'e', 'i', 'o', 'u', 'n'});
partes = regexprep(lower(strsplit(nombre, '/')), '[^a-z0-9]+', '_');
partes = regexprep(partes, '^_+|_+$', '');
nombre = strjoin(partes, '-');
end
