% CONTROL SEMI-AUTOM�TICO COORDINADO DE GR�A PORTUARIA DE MUELLE

% Par�metros del modelo (eje x, horizontal, x=0 en borde del muelle)
% Traslaci�n del carro
x_min = -30.0;  % [m] posici�n m�nima (sobre muelle)
x_max = 50.0;   % [m] posici�n m�xima (sobre barco)
x_d_max = 4.0;  % [m/s] velocidad m�xima (cargado o sin carga)
x_dd_max = 1.0; % [m/s^2] aceleraci�n m�xima (cargado o sin carga)

% Izaje de carga (eje y, vertical, y=0 al nivel del muelle)
y_min = -20.0;  % [m] posici�n m�nima (dentro de barco)
y_max = 40.0;   % [m] posici�n m�xima (sobre barco)
y_t0 = 45.0;    % [m] altura (fija) de carro y sistema de izaje
y_sb = 15.0;    % [m] despeje m�nimo sobre borde de muelle (sill beam)
y_d_maxn = 1.5; % [m/s] velocidad m�xima (cargado con carga nominal)
y_d_max0 = 3.0; % [m/s] velocidad m�xima (sin carga)
y_dd_max = 1.0; % [m/s^2] aceleraci�n m�xima (cargado o sin carga)

%...
