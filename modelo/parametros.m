% CONTROL SEMI-AUTOMÁTICO COORDINADO DE GRÚA PORTUARIA DE MUELLE

% Parámetros del modelo (eje x, horizontal, x=0 en borde del muelle)
% Traslación del carro
x_min = -30.0;  % [m] posición mínima (sobre muelle)
x_max = 50.0;   % [m] posición máxima (sobre barco)
x_d_max = 4.0;  % [m/s] velocidad máxima (cargado o sin carga)
x_dd_max = 1.0; % [m/s^2] aceleración máxima (cargado o sin carga)

% Izaje de carga (eje y, vertical, y=0 al nivel del muelle)
y_min = -20.0;  % [m] posición mínima (dentro de barco)
y_max = 40.0;   % [m] posición máxima (sobre barco)
y_t0 = 45.0;    % [m] altura (fija) de carro y sistema de izaje
y_sb = 15.0;    % [m] despeje mínimo sobre borde de muelle (sill beam)
y_d_maxn = 1.5; % [m/s] velocidad máxima (cargado con carga nominal)
y_d_max0 = 3.0; % [m/s] velocidad máxima (sin carga)
y_dd_max = 1.0; % [m/s^2] aceleración máxima (cargado o sin carga)

%...
