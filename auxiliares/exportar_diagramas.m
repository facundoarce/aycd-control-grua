function exportar_diagramas(dir_salida, modelo, formatos)
%EXPORTAR_DIAGRAMAS Exporta los diagramas del modelo como imágenes vectoriales.
%   exportar_diagramas
%   exportar_diagramas(dir_salida)
%   exportar_diagramas(dir_salida, modelo)
%   exportar_diagramas(dir_salida, modelo, formatos)
%       Exporta el nivel superior y cada subsistema de modelo (por defecto
%       sistema_completo) en formato vectorial (texto y líneas vectoriales, sin
%       pérdida de resolución al hacer zoom). Prueba los formatos en orden y
%       usa el primero que acepte la versión de Simulink (R2019a no acepta
%       'epsc'). Por defecto formatos = {'svg', 'pdf', 'eps', 'psc'}: el SVG
%       conserva el texto como texto y, convertido con rsvg-convert, pesa hasta
%       ~15 veces menos que el PDF de Simulink (que dibuja parte del texto como
%       curvas). P. ej. exportar_diagramas('', '', {'pdf'}) exporta sólo en PDF.
%       Los charts de Stateflow se exportan con sfprint si print -s falla.
%       Por defecto dir_salida es figuras/simulink/originales del informe, que
%       contiene a este repositorio como submódulo en ref/aycd-control-grua.
%       No se exportan los bloques MATLAB Function (su código va en los
%       anexos) ni los bloques enmascarados (Signal Builder).
%       Durante la exportación se desactivan las miniaturas del contenido de
%       los subsistemas (ContentPreviewEnabled): son ilegibles al tamaño del
%       informe y multiplican por ~20 el tamaño del archivo.
%       Escribe además indice.txt con la correspondencia archivo - bloque.
%       No modifica ni guarda el modelo.
%       En el informe, convertir a PDF recortado con figuras/scripts/recortar_diagramas.sh.

if nargin < 2 || isempty(modelo)
    modelo = 'sistema_completo';
end
if nargin < 3 || isempty(formatos)
    formatos = {'svg', 'pdf', 'eps', 'psc'};
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

% Sin miniaturas del contenido; se restauran al final
estaba_sucio = get_param(modelo, 'Dirty');
con_miniatura = find_system(modelo, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
    'Variants', 'AllVariants', 'ContentPreviewEnabled', 'on');
set_miniatura(con_miniatura, 'off');

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
    [ext, msg] = imprimir(s, base, strcmp(sf_tipo, 'Chart'), formatos);
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

set_miniatura(con_miniatura, 'on');
set_param(modelo, 'Dirty', estaba_sucio);
if ~estaba_cargado
    close_system(modelo, 0);
end
fprintf('%d diagramas exportados en %s\n', n_ok, dir_salida);
end

function [ext, msg] = imprimir(s, base, es_chart, formatos)
% Imprime s en el primer formato vectorial aceptado. Devuelve la extensión
% del archivo escrito, o vacío y el último mensaje de error.
% Formato de print/sfprint -> extensión
EXTENSIONES = containers.Map({'pdf', 'svg', 'eps', 'psc'}, {'.pdf', '.svg', '.eps', '.ps'});
ext = '';
msg = '';
for k = 1:numel(formatos)
    archivo = [base EXTENSIONES(formatos{k})];
    try
        print(['-s' s], ['-d' formatos{k}], archivo);
    catch err
        msg = err.message;
        if ~es_chart
            continue
        end
        try
            sfprint(s, formatos{k}, archivo);
        catch err
            msg = err.message;
            continue
        end
    end
    ext = EXTENSIONES(formatos{k});
    return
end
end

function set_miniatura(bloques, valor)
% ContentPreviewEnabled de cada bloque (los bloques de biblioteca pueden no
% aceptarlo)
for k = 1:numel(bloques)
    try
        set_param(bloques{k}, 'ContentPreviewEnabled', valor);
    catch
    end
end
end

function nombre = nombre_archivo(ruta)
% Ruta del bloque -> nombre de archivo ASCII corto: los dos últimos niveles
% separados por '-', sin artículos ni preposiciones (rutas de Windows de menos
% de 260 caracteres, necesario para git)
nombre = strrep(ruta, newline, ' ');
nombre = regexprep(nombre, {'[áÁ]', '[éÉ]', '[íÍ]', '[óÓ]', '[úÚ]', '[ñÑ]'}, ...
    {'a', 'e', 'i', 'o', 'u', 'n'});
partes = strsplit(lower(nombre), '/');
partes = partes(max(1, end-1):end);
for k = 1:numel(partes)
    palabras = regexp(partes{k}, '[a-z0-9]+', 'match');
    palabras = palabras(~ismember(palabras, {'de', 'del', 'en', 'la', 'el'}));
    partes{k} = strjoin(palabras, '_');
end
nombre = strjoin(partes, '-');
end
