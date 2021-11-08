% CONTROL SEMI-AUTOMÁTICO COORDINADO DE GRÚA PORTUARIA DE MUELLE

% Parámetros del modelo
m_c = 50000.0;  % [kg] masa del carro (incluye sistema de izaje)

% Traslación del carro (eje x, horizontal, x=0 en borde del muelle)
R_w = 0.5;      % [m] radio primitivo de rueda
J_w = 2.0;      % [kg.m^2] momento de inercia de ruedas (en eje lento)
i_c = 15.0;     % [adimensional] relación de reducción de caja reductora
J_m_c = 10.0;   % [kg.m^2] momento de inercia de motor y freno (en eje rápido)
b_eq_c = 30.0;  % [N.m/(rad/s)] fricción mécanica equivalente
x_0 = -20.0;      % [m] posición inicial
x_min = -30.0;  % [m] posición mínima (sobre muelle)
x_max = 50.0;   % [m] posición máxima (sobre barco)
x_d_max = 4.0;  % [m/s] velocidad máxima (cargado o sin carga)
x_dd_max = 1.0; % [m/s^2] aceleración máxima (cargado o sin carga)

J_eq_c = m_c + ( J_w + i_c^2*J_m_c ) / R_w^2;

% Izaje de carga (eje y, vertical, y=0 al nivel del muelle)
y_min = -20.0;  % [m] posición mínima (dentro de barco)
y_max = 40.0;   % [m] posición máxima (sobre barco)
y_t0 = 45.0;    % [m] altura (fija) de carro y sistema de izaje
y_sb = 15.0;    % [m] despeje mínimo sobre borde de muelle (sill beam)
y_d_maxn = 1.5; % [m/s] velocidad máxima (cargado con carga nominal)
y_d_max0 = 3.0; % [m/s] velocidad máxima (sin carga)
y_dd_max = 1.0; % [m/s^2] aceleración máxima (cargado o sin carga)

%...