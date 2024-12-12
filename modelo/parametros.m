% CONTROL SEMI-AUTOMÁTICO COORDINADO DE GRÚA PORTUARIA DE MUELLE
clc;
clc;
clear all;

% Parámetros del modelo
g = 9.80665;    % [m/s^2] aceleración de la gravedad

%% Traslación del carro (eje x, horizontal, x=0 en borde del muelle)

% Parámetros traslación (w de wheel, t de trolley)
m_t = 50000.0;  % [kg] masa del carro (incluye sistema de izaje)
R_w = 0.5;      % [m] radio primitivo de rueda
J_w = 2.0;      % [kg.m^2] momento de inercia de ruedas (en eje lento)
i_t = 15.0;     % [adimensional] relación de reducción de caja reductora
J_mt = 10.0;    % [kg.m^2] momento de inercia de motor y freno (en eje rápido)
b_eqt = 30.0;   % [N.m/(rad/s)] fricción mécanica equivalente

% Variables traslación:
x_t0 = -20.0;       % [m] posición inicial
x_tmin = -30.0;     % [m] posición mínima (sobre muelle)
x_tmax = 50.0;      % [m] posición máxima (sobre barco)
x_d_tmax = 4.0;     % [m/s] velocidad máxima (cargado o sin carga)
x_dd_tmax = 0.8;    % [m/s^2] aceleración máxima (cargado o sin carga)

m_eqt = m_t + ( J_w + i_t^2*J_mt ) / R_w^2;

%% Izaje de carga (eje y, vertical, y=0 al nivel del muelle)

% Parámetros izaje:
y_min = -20.0;      % [m] posición mínima (dentro de barco)
y_max = 40.0;       % [m] posición máxima (sobre barco)
y_t0 = 45.0;        % [m] altura (fija) de carro y sistema de izaje
y_sb = 15.0;        % [m] despeje mínimo sobre borde de muelle (sill beam)
y_d_maxn = 1.5;     % [m/s] velocidad máxima (cargado con carga nominal)
y_d_max0 = 3.0;     % [m/s] velocidad máxima (sin carga)
y_dd_max = 0.75;    % [m/s^2] aceleración máxima (cargado o sin carga)
l_0h = 0.0;         % [m] posición inicial (REVISAR NOMBRE)
R_d = 0.75;         % [m] diámetro de la rueda del tambor
J_d = 8.0;          % [kg.m^2] momento de inercia del tambor de izaje
i_h = 30.0;         % [] relación de transmisión de motor de izaje a tambor
J_mh = 30.0;        % [kg.m^2] momento de inercia del motor de izaje
b_eqh = 18.0;       % [N.m.s/rad] amortiguamiento viscoso de motor de izaje
%bd = ?
%bmh = ?

% Variables izaje:
J_eqi = J_d + J_mh * i_h^2;
J_eqh = J_eqi / R_d^2;
% b_eqi=bd+bmh*ih^2;
% b_eqh=b_eqi/Rd^2;

%% Carga suspendida

m_l0 = 15000.0;             % [kg] masa de la carga con spreader vacío (sin carga)
m_cont_0 = 2000.0;          % [kg] masa del container vacío
m_cont_nom = 50000.0;       % [kg] masa nominal del container cargado
m_lmin = m_l0 + m_cont_0;   % [kg] masa mínima de la carga con spreader con carga
m_lnom = m_l0 + m_cont_nom; % [kg] masa nominal de la carga con spreader con carga
m_l = (m_lmin + m_lnom)/2;  % [kg] masa de la carga  !!!REVISAR!!!

x_l0 = 0.0;                 % [m] posición inicial de la carga en x  !!!REVISAR!!!
y_l0 = 0.0;                 % [m] posición inicial de la carga en y  !!!REVISAR!!!

% Parámetros de contacto para carga apoyada
K_cy = 1.3e6;   % [kN/m] rigidez vertical (compresión)
b_cy = 500.0;   % [kN/(m/s)] fricción vertical (compresión)
b_cx = 1000.0;  % [kN/(m/s)] fricción horizontal (arrastre)

% Parámetros del cable de acero
K_w = 1800.0;   % [kN/m] rigidez del cable (tracción)
b_w = 30.0;     % [kN/(m/s)] fricción interna del cable


%% Controladores de movimiento

% Movimiento de traslación del carro
% Función de transferencia: H_t(s)=V_t(s)/T_mt(s)=(i_t/R_w)/(m_eqt*s+b_eqt)
s_t = -b_eqt/m_eqt;     % [] polo del subsistema
w_t = -s_t;             % [] frecuencia natural del subsistema

% Controlador de movimiento de traslación del carro
% Método de sintonía serie
w_pos_t = 5 * w_t;      % [] frecuencia del controlador
n_t = 2.5;
ba_t = m_eqt * n_t * w_pos_t;
Ksa_t = m_eqt * n_t * w_pos_t^2;
Ksia_t = m_eqt * w_pos_t^3;

% Movimiento de izaje del carro
% Función de transferencia: H_h(s)=V_h(s)/T_mh(s)=(i_h/R_d)/(J_eqh*s+b_eqh)
s_h = -b_eqh/J_eqh;     % [] polo del subsistema
w_h = -s_h;             % [] frecuencia natural del subsistema

% Controlador de movimiento de izaje de carga
% Método de sintonía serie
w_pos_h = 5 * w_h;      % [] frecuencia del controlador
n_h = 2.5;
ba_h = J_eqh * n_h * w_pos_h;
Ksa_h = J_eqh * n_h * w_pos_h^2;
Ksia_h = J_eqh * w_pos_h^3;