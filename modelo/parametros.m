%% CONTROL SEMI-AUTOMÁTICO COORDINADO DE GRÚA PORTUARIA DE MUELLE
% Autores: Arce Facundo, Cantaloube Adrián
clc;
clear variables;

%% SISTEMA DE IZAJE
Y_t0 = 45.0;        % [m] altura (fija) de poleas de suspensión de izaje en el carro

% Cable de acero de izaje (parámetros unitarios)
% w: wirerope | u: unit
k_wu = 2.36e8;      % [(N/m).m] rigidez unitaria de izaje (tracción)
b_wu = 150;         % [(N/(m/s))/m] fricción unitaria de izaje (tracción)

% Accionamiento de sistema de izaje
% h: hoist | hd: | hEb: hoist emergency break | hm: hoist motor | hb: hoist break |
% Eje rápido: motor + disco de freno de operación + etapa de entrada de caja reductora
% Eje lento: tambor + disco de freno de emergencia + etapa de salida de caja reductora
r_hd = 0.75;        % [m] radio primitivo de tambor
J_hd_hEb = 3800;    % [kg.m^2] momento de inercia equivalente de eje lento
b_hd = 8.0;         % [N.m/(rad/s)] coeficiente de fricción mecánica viscosa equivalente del eje lento
b_hEb = 2.2e9;      % [N.m/(rad/s)] coeficiente de fricción mecánica viscosa equivalente del freno de emergencia
T_hEb_max = 1.1e6;  % [N.m] torque máximo de frenado del freno de emergencia
i_h = 22.0;         % relación de transmisión total de caja reductora de engranajes
J_hm_hb = 30.0;     % [kg.m^2] momento de inercia equivalente de eje rápido
b_hm = 18.0;        % [N.m/(rad/s)] coeficiente de fricción mecánica viscosa equivalente del eje rápido
b_hb = 1.0e8;       % [N.m/(rad/s)] coeficiente de fricción mecánica viscosa equivalente del freno de operación
T_hb_max = 5.0e4;   % [N.m] torque máximo de frenado del freno de operación
tau_hm = 1.0;       % [ms] constante de tiempo de modulador de torque
T_hm_max = 2.0e4;   % [N.m] torque máximo de motorización / frenado regenerativo del motor

% Modelo de izaje equivalente
% m_h_eq * v_h_dot = F_hm_eq - b_h_eq * v_h - F_hw
m_h_eq = 2 * ( J_hd_hEb + i_h^2*J_hm_hb ) / (r_hd^2);   % [m] masa equivalente del modelo de izaje
b_h_eq = 2 * ( b_hd + i_h^2*b_hm ) / (r_hd^2);          % [N/(m/s)] coeficiente de fricción equivalente del modelo de izaje
% Altura de la carga sin balanceo  ?_h = Y_t0 - l_h : [-20.0 (dentro de barco) … 0.0 (sobre barco/muelle) … +40.0] m
y_h0 = 0.0;         % [m] altura inicial del accionamiento de izaje: 0.0 m (sobre el barco/muelle)

%% SISTEMA DE TRASLACIÓN DE CARRO
% Carro y cable de acero de carro
% t: trolley | w: wirerope
M_t = 30000;        % [kg] masa equivalente de carro
b_t = 90.0;         % [N/(m/s)] coeficiente de fricción mecánica viscosa equivalente de carro
K_tw = 4.8e5;       % [N/m] rigidez equivalente total a tracción de cable tensado de carro
b_tw = 3.0e3;       % [N/(m/s)] fricción interna o amortiguamiento de cable tensado de carro

% Posición horizontal del carro  x_t : [-30.0 (sobre muelle) … 0.0 … (sobre barco) +50.0] m
x_t0 = -30.0;       % [m] posición horizontal inicial del carro (sobre muelle)

% Accionamiento de traslación de carro
% t: trolley | td: trolley drum? | tm: trolley motor | tb: trolley break |
% Eje rápido: motor + disco de freno de operación + etapa de entrada de caja reductora
% Eje lento: tambor + etapa de salida de caja reductora
r_td = 0.50;        % [m] radio primitivo de tambor
J_td = 1200;        % [kg.m^2] momento de inercia equivalente de eje lento
b_td = 1.8;         % [N.m/(rad/s)] coeficiente de fricción mecánica viscosa equivalente del eje lento
i_t = 30.0;         % relación de transmisión total de caja reductora de engranajes
J_tm_tb = 7.0;      % [kg.m^2] momento de inercia equivalente de eje rápido
b_tm = 6.0;         % [N.m/(rad/s)] coeficiente de fricción mecánica viscosa equivalente del eje rápido
b_tb = 5.0e6;       % [N.m/(rad/s)] coeficiente de fricción mecánica viscosa equivalente del freno de operación
T_tb_max = 5.0e3;   % [N.m] torque máximo de frenado del freno de operación
tau_tm = 1.0;       % [ms] constante de tiempo de modulador de torque
T_tm_max = 3.0e3;   % [N.m] torque máximo de motorización / frenado regenerativo del motor

% Modelo de traslación equivalente
% Modelo equivalente del tambor
% m_td_eq * v_td_dot = F_tdm_eq - b_td_eq * v_td - F_tw
m_td_eq = ( J_td + i_t^2*J_tm_tb ) / (r_td^2);    % [m] masa equivalente del modelo de traslación del tambor del carro
b_td_eq = ( b_td + i_t^2*b_tm ) / (r_td^2);       % [N/(m/s)] coeficiente de fricción equivalente del tambor del carro
x_td0 = x_t0;       % [m] posición horizontal inicial del tambor del carro


%% MOVIMIENTO DE LA CARGA
% c: container | s: spreader
g = 9.80665;        %% [m/s^2] aceleración de la gravedad
H_c = 2.5;          %% [m] alto y ancho de container estándar
M_s = 15000;        %% [kg] masa de spreader + headblock (sin container)
M_c_max = 50000;    %% [kg] masa de container máxima (totalmente cargado)
M_c_min = 2000;     %% [kg] masa de container mínima (vacío, sin carga)

% Parámetros de contacto para carga apoyada
% c: contact
K_cy = 1.8e9;       % [N/m] rigidez  por contacto vertical (compresión)
b_cy = 1.0e7;       % [N/(m/s)] fricción por contacto vertical (compresión)
b_cx = 1.0e6;       % [N/(m/s)] fricción por contacto horizontal (arrastre)

% Estado inicial de la carga
x_l0 = x_t0;        % [m] posición horizontal inicial de la carga (igual a posición inicial del carro sin balanceo)
y_l0 = y_h0;        % [m] posición vertical inicial de la carga  (igual a altura de la carga sin balanceo)


%% Controladores de movimiento

% Movimiento de izaje del carro
% Función de transferencia: H_h(s)=V_h(s)/T_mh(s)=(i_h/R_d)/(J_eqh*s+b_eqh)
s_h = -b_eqh/J_eqh;     % [] polo del subsistema
w_h = -s_h;             % [] frecuencia natural del subsistema

% Controlador de movimiento de izaje de carga
% Método de sintonía serie
% tau_hm = 1.0;           % [ms] constante de tiempo de modulador de torque en motor-drive de izaje



w_pos_h = 5 * w_h;      % [] frecuencia del controlador
n_h = 2.5;
ba_h = J_eqh * n_h * w_pos_h;
Ksa_h = J_eqh * n_h * w_pos_h^2;
Ksia_h = J_eqh * w_pos_h^3;




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




% %% Traslación del carro (eje x, horizontal, x=0 en borde del muelle)
% 
% % Parámetros traslación (w de wheel, t de trolley)
% 
% 
% 
% m_t = 50000.0;      % [kg] masa del carro (incluye sistema de izaje)
% R_w = 0.5;          % [m] radio primitivo de rueda
% J_w = 2.0;          % [kg.m^2] momento de inercia de ruedas (en eje lento)
% i_t = 15.0;         % [adimensional] relación de reducción de caja reductora
% J_mt = 10.0;        % [kg.m^2] momento de inercia de motor y freno (en eje rápido)
% b_eqt = 30.0;       % [N.m/(rad/s)] fricción mécanica equivalente
% 
% % Variables traslación:
% x_t0 = -20.0;       % [m] posición inicial
% x_tmin = -30.0;     % [m] posición mínima (sobre muelle)
% x_tmax = 50.0;      % [m] posición máxima (sobre barco)
% x_d_tmax = 4.0;     % [m/s] velocidad máxima (cargado o sin carga)
% 
% x_dd_tmax = 0.8;    % [m/s^2] aceleración máxima (cargado o sin carga)
% 
% m_eqt = m_t + ( J_w + i_t^2*J_mt ) / R_w^2;
% 
% %% Izaje de carga (eje y, vertical, y=0 al nivel del muelle)
% 
% % Parámetros izaje:
% Y_t0 = 45.0;        %% [m] altura (fija) de carro y sistema de izaje
% y_min = -20.0;      % [m] posición mínima (dentro de barco)
% y_max = 40.0;       % [m] posición máxima (sobre barco)
% y_sb = 15.0;        % [m] despeje mínimo sobre borde de muelle (sill beam)
% y_d_maxn = 1.5;     % [m/s] velocidad máxima (cargado con carga nominal)
% y_d_max0 = 3.0;     % [m/s] velocidad máxima (sin carga)
% y_dd_max = 0.75;    % [m/s^2] aceleración máxima (cargado o sin carga)
% l_0h = 0.0;         % [m] posición inicial (REVISAR NOMBRE)
% R_d = 0.75;         % [m] diámetro de la rueda del tambor
% J_d = 8.0;          % [kg.m^2] momento de inercia del tambor de izaje
% i_h = 30.0;         % [] relación de transmisión de motor de izaje a tambor
% J_mh = 30.0;        % [kg.m^2] momento de inercia del motor de izaje
% b_eqh = 18.0;       % [N.m.s/rad] amortiguamiento viscoso de motor de izaje
% %bd = ?
% %bmh = ?
% 
% % Variables izaje:
% J_eqi = J_d + J_mh * i_h^2;
% J_eqh = J_eqi / R_d^2;
% % b_eqi=bd+bmh*ih^2;
% % b_eqh=b_eqi/Rd^2;
% 
% %% Carga suspendida
% 
% m_l0 = 15000.0;             % [kg] masa de la carga con spreader vacío (sin carga)
% m_cont_0 = 2000.0;          % [kg] masa del container vacío
% m_cont_nom = 50000.0;       % [kg] masa nominal del container cargado
% m_lmin = m_l0 + m_cont_0;   % [kg] masa mínima de la carga con spreader con carga
% m_lnom = m_l0 + m_cont_nom; % [kg] masa nominal de la carga con spreader con carga
% m_l = (m_lmin + m_lnom)/2;  % [kg] masa de la carga  !!!REVISAR!!!
% 
% x_l0 = 0.0;                 % [m] posición inicial de la carga en x  !!!REVISAR!!!
% y_l0 = 0.0;                 % [m] posición inicial de la carga en y  !!!REVISAR!!!
% 
% % Parámetros de contacto para carga apoyada
% K_cy = 1.3e6;   % [kN/m] rigidez vertical (compresión)
% b_cy = 500.0;   % [kN/(m/s)] fricción vertical (compresión)
% b_cx = 1000.0;  % [kN/(m/s)] fricción horizontal (arrastre)
% 
% % Parámetros del cable de acero
% K_w = 1800.0;   % [kN/m] rigidez del cable (tracción)
% b_w = 30.0;     % [kN/(m/s)] fricción interna del cable
% 
% 
% %% Controladores de movimiento
% 
% % Movimiento de traslación del carro
% % Función de transferencia: H_t(s)=V_t(s)/T_mt(s)=(i_t/R_w)/(m_eqt*s+b_eqt)
% s_t = -b_eqt/m_eqt;     % [] polo del subsistema
% w_t = -s_t;             % [] frecuencia natural del subsistema
% 
% % Controlador de movimiento de traslación del carro
% % Método de sintonía serie
% w_pos_t = 5 * w_t;      % [] frecuencia del controlador
% n_t = 2.5;
% ba_t = m_eqt * n_t * w_pos_t;
% Ksa_t = m_eqt * n_t * w_pos_t^2;
% Ksia_t = m_eqt * w_pos_t^3;
% 
% % Movimiento de izaje del carro
% % Función de transferencia: H_h(s)=V_h(s)/T_mh(s)=(i_h/R_d)/(J_eqh*s+b_eqh)
% s_h = -b_eqh/J_eqh;     % [] polo del subsistema
% w_h = -s_h;             % [] frecuencia natural del subsistema
% 
% % Controlador de movimiento de izaje de carga
% % Método de sintonía serie
% w_pos_h = 5 * w_h;      % [] frecuencia del controlador
% n_h = 2.5;
% ba_h = J_eqh * n_h * w_pos_h;
% Ksa_h = J_eqh * n_h * w_pos_h^2;
% Ksia_h = J_eqh * w_pos_h^3;