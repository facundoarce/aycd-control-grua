%% CONTROL SEMI-AUTOMÁTICO COORDINADO DE GRÚA PORTUARIA DE MUELLE
% Autores: Arce Facundo, Cantaloube Adrián
clc;
clear variables;
T_s0 = 20/1000;     % [s] Tiempo de muestreo de sistema de control nivel 0 (control de seguridad)
T_s1 = 20/1000;     % [s] Tiempo de muestreo de sistema de control nivel 1 (control supervisor)
T_s2 = 1/1000;      % [s] Tiempo de muestreo de sistema de control nivel 2 (control regulatorio)
T_anim = 200/1000;  % [s] Período de refresco de la animación de la grúa (sólo visualización)
modo_comandos = 1;  % Comandos de operador: 1 = joystick; 2 = escenario (cmd_* en el workspace, ver correr_escenario.m)

%% SISTEMA DE IZAJE
Y_t0 = 45.0;        % [m] altura (fija) de poleas de suspensión de izaje en el carro
Y_sb = 5.0;         % [m] despeje mínimo sobre borde de muelle
H_c = Simulink.Parameter(2.59);  % [m] alto de container estándar
H_c.CoderInfo.StorageClass = 'SimulinkGlobal';
W_c = Simulink.Parameter(2.44);  % [m] ancho de container estándar
W_c.CoderInfo.StorageClass = 'SimulinkGlobal';

% Cable de acero de izaje (parámetros unitarios)
% w: wirerope | u: unit
k_wu = 2.36e8;      % [(N/m).m] rigidez unitaria de izaje (tracción)
b_wu = 150;         % [(N/(m/s))/m] fricción unitaria de izaje (tracción)
L_h0 = 110;         % [m] longitud de despliegue fijo de wirerope de izaje

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

% Velocidad de izaje nominal y máxima (Fig. 5)
v_h_nom = 1.5;      % [m/s] velocidad de izaje máxima para carga suspendida nominal m_l_nom = 65000 kg
v_h_max = 3.0;      % [m/s] velocidad de izaje máxima para carga suspendida m_l_0 = 15000 kg a m_l_max@v_h_max = 32500 kg
P_h_nom = 956150;   % [W] Potencia nominal de izaje (constante) entre v_h_nom y v_h_max
% P_h_nom = m_l * g * v_h => v_h = P_h_nom / ( m_l * g ) 
% Ejemplo: v_h(32500) = 956150 / ( 32500 * 9.80665 ) = 3.0 = v_h_max
%          v_h(65000) = 956150 / ( 65000 * 9.80665 ) = 1.5 = v_h_nom

% Aceleración de izaje máxima
a_h_max = 0.75;  % [m/s^2] aceleración de izaje máxima (cargado o sin carga)

% Modulador de torque en motor-drive de izaje con limitador de torque máx.
tau_hm = 1.0e-3;    % [s] constante de tiempo de modulador de torque
T_hm_MAX = 2.0e4;   % [N.m] torque máximo de motorización / frenado regenerativo del motor
% (Ec 6.b) 2 * v_h(t) = r_hd * w_hd(t)
% (Ec 6.d) w_hd(t) * i_h = w_hm(t)
% => w_hm = 2 * (i_h / r_hd) * v_h(t)
w_hm_rated = 2 * (i_h / r_hd) * v_h_nom;  % [rad/s] velocidad del motor de izaje equivalente para velocidad de izaje nominal
eps_w_hm = 0.1;     % [rad/s] epsilon de velocidad del motor de izaje para evitar división por cero

% Posición de fines de carrera de izaje (rotativos en tambor)
y_h_min_oper = Simulink.Parameter(-20.0);  % [m] límite de operación mínimo (dentro del barco)
y_h_min_oper.CoderInfo.StorageClass = 'SimulinkGlobal';
y_h_max_oper = Simulink.Parameter(40.0);   % [m] límite de operación máximo (sobre barco/muelle)
y_h_max_oper.CoderInfo.StorageClass = 'SimulinkGlobal';
y_h_min_emer = y_h_min_oper.Value - 0.5;   % [m] límite de emergencia mínimo (guía 3.2: 0.50 m más allá del de operación)
y_h_max_emer = y_h_max_oper.Value + 0.5;   % [m] límite de emergencia máximo

% theta_hd_min_oper = -2*(Y_t0 - y_h_min_oper)/r_hd;   % [rad] límite de operación mínimo (rotativos en tambor)
% theta_hd_max_oper = -2*(Y_t0 - y_h_max_oper)/r_hd;   % [rad] límite de operación máximo (rotativos en tambor)
% theta_hd_min_emer = -2*(Y_t0 - y_h_min_emer)/r_hd;   % [rad] límite de emergencia mínimo (rotativos en tambor)
% theta_hd_max_emer = -2*(Y_t0 - y_h_max_emer)/r_hd;   % [rad] límite de emergencia máximo (rotativos en tambor)

% Modelo de izaje equivalente (fuerza total de los dos ramales de cable)
% m_h_eq * l_h_ddot = 2*F_hw - F_h_eq - b_h_eq * l_h_dot, con F_h_eq = 2*i_h*T_hm/r_hd
m_h_eq = 4 * ( J_hd_hEb + i_h^2*J_hm_hb ) / (r_hd^2);  % [m] masa equivalente del modelo de izaje
b_h_eq = 4 * ( b_hd + i_h^2*b_hm ) / (r_hd^2);         % [N/(m/s)] coeficiente de fricción equivalente del modelo de izaje

%% SISTEMA DE TRASLACIÓN DE CARRO
% Carro y cable de acero de carro
% t: trolley | w: wirerope
M_t = Simulink.Parameter(30000);   % [kg] masa equivalente de carro
M_t.CoderInfo.StorageClass = 'SimulinkGlobal';
b_t = 90.0;         % [N/(m/s)] coeficiente de fricción mecánica viscosa equivalente de carro
K_tw = 4.8e5;       % [N/m] rigidez equivalente total a tracción de cable tensado de carro
b_tw = 3.0e3;       % [N/(m/s)] fricción interna o amortiguamiento de cable tensado de carro

% Posición horizontal del carro  x_t : [-30.0 (sobre muelle) … 0.0 … (sobre barco) +50.0] m
x_t0 = W_c.Value / 2; % [m] posición horizontal inicial del carro (sobre barco)

% Posición de fines de carrera de traslación del carro (fijos sobre viga)
x_t_min_oper = Simulink.Parameter(-30.0);  % [m] límite de operación mínimo (sobre muelle)
x_t_min_oper.CoderInfo.StorageClass = 'SimulinkGlobal';
x_t_max_oper = Simulink.Parameter(50.0);   % [m] límite de operación máximo (sobre barco)
x_t_max_oper.CoderInfo.StorageClass = 'SimulinkGlobal';
x_t_min_emer = x_t_min_oper.Value - 0.5;   % [m] límite de emergencia mínimo (guía 3.2: 0.50 m más allá del de operación)
x_t_max_emer = x_t_max_oper.Value + 0.5;   % [m] límite de emergencia máximo

% Velocidad y aceleración máximas
v_t_max = 4.0;  % [m/s] velocidad de traslación máxima (cargado o sin carga)
a_t_max = 0.8;  % [m/s^2] aceleración de traslación máxima (cargado o sin carga)
k_v_lim = 0.8;  % [1/s] pendiente máxima de la curva de frenado hacia los fines de carrera del carro (v <= min(sqrt(2*a_t_max*d), k_v_lim*d); tramo lineal a menos de 2*a_t_max/k_v_lim^2 = 2.5 m)

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
tau_tm = 1.0e-3;    % [s] constante de tiempo de modulador de torque
T_tm_max = 4.0e3;   % [N.m] torque máximo de motorización / frenado regenerativo del motor

% Modelo de traslación equivalente
% Referido al sistema de referencia del tambor del carro
% m_td_eq * v_td_dot = F_tdm_eq - b_td_eq * v_td - F_tw
m_td_eq = ( J_td + i_t^2*J_tm_tb ) / (r_td^2);   % [m] masa equivalente del modelo de traslación del tambor del carro
b_td_eq = ( b_td + i_t^2*b_tm ) / (r_td^2);      % [N/(m/s)] coeficiente de fricción equivalente del tambor del carro
x_td0 = x_t0;       % [m] posición horizontal inicial del tambor del carro

% Modelo de traslación equivalente
% Referido al sistema de referencia del carro
% m_t_eq * x_t_ddot = F_td_eq - b_t_eq * x_t_dot - F_tl_eq
m_t_eq = M_t.Value + ( J_td + i_t^2*J_tm_tb ) / (r_td^2);  % [m] masa equivalente del modelo de traslación del carro
b_t_eq = b_t + ( b_td + i_t^2*b_tm ) / (r_td^2);           % [N/(m/s)] coeficiente de fricción equivalente del carro
b_t_eq = Simulink.Parameter(b_t_eq);
b_t_eq.CoderInfo.StorageClass = 'SimulinkGlobal';

%% Perfil de obstáculos
% Grilla de slots de ancho W_c alineada con x = 0 (borde muelle/barco), de modo que ningún slot quede repartido entre muelle y barco
x_slot0 = Simulink.Parameter(-ceil(-x_t_min_oper.Value / W_c.Value) * W_c.Value);  % [m] borde izquierdo del primer slot (<= x_t_min_oper)
x_slot0.CoderInfo.StorageClass = 'SimulinkGlobal';

% Número de slots necesarios para cubrir el rango de operación
N_slots = Simulink.Parameter(ceil((x_t_max_oper.Value - x_slot0.Value) / W_c.Value));
N_slots.CoderInfo.StorageClass = 'SimulinkGlobal';

% Valores iniciales sin ningún contenedor
y_c0_quay = 0;    % [m] altura base del muelle
y_c0_ship = -20;  % [m] altura base dentro del barco

% Vector inicial del perfil de obstáculos (dividido en slots)
x_slots = x_slot0.Value + ((0:N_slots.Value-1) + 0.5) * W_c.Value;  % [m] centro de cada slot (evita ambigüedad numérica en x = 0)
y_c0_ini = zeros(1, N_slots.Value);
y_c0_ini(x_slots <  0) = y_c0_quay;  % altura base lado muelle (x < 0)
y_c0_ini(x_slots >= 0) = y_c0_ship;  % altura base lado barco  (x >= 0)
random_obstacles = false;  % si true, inicializa perfil de obstáculos con contenedores aleatoreos
if random_obstacles
    n_max_stack = 10; % Número aleatorio de contenedores apilados por slot (0 a 10)
    y_c0_ini = y_c0_ini + randi([0, n_max_stack], 1, N_slots.Value) * H_c.Value;
else
    y_c0_ini = y_c0_ini + ones(1, N_slots.Value) * H_c.Value; % al menos un contenedor en cada slot
end
y_c0_0 = Simulink.Parameter(y_c0_ini);  % [m] perfil de obstáculos inicial (vector de N_slots elementos)
y_c0_0.CoderInfo.StorageClass = 'SimulinkGlobal';

% Altura del perfil de obstáculos en la posición inicial del carro
slot_x_t0 = floor((x_t0 - x_slot0.Value) / W_c.Value) + 1;
y_c0_x_l0 = Simulink.Parameter(y_c0_0.Value(slot_x_t0));  % [m]
y_c0_x_l0.CoderInfo.StorageClass = 'SimulinkGlobal';

% Altura de la carga sin balanceo  y_h = Y_t0 - l_h : [-20.0 (dentro de barco) … 0.0 (sobre barco/muelle) … +40.0] m
y_h0 = y_c0_x_l0.Value;  % + H_c.Value; % [m] altura inicial del accionamiento de izaje: apoyado sobre perfil de obstáculos
% y_h0 = Y_t0 - l_h0;

clearance_min = 2;  % [m] distancia de despeje mínima entre perfil de obstáculos y carga

%% MOVIMIENTO DE LA CARGA
% c: container | s: spreader
g = Simulink.Parameter(9.80665);  %% [m/s^2] aceleración de la gravedad
g.CoderInfo.StorageClass = 'SimulinkGlobal';
M_s = 15000;        %% [kg] masa de spreader + headblock (sin container)
M_c_max = 50000;    %% [kg] masa de container máxima (totalmente cargado)
M_c_min = 2000;     %% [kg] masa de container mínima (vacío, sin carga)
overload = false;   % [bool] condición de sobrecarga para simulación. Setear en "true" para simular M_cx > M_c_max

% Umbral de carga apoyada/suspendida (supervisor nivel 1 y controladores nivel 2)
F_hw_min = M_s * g.Value / 2;  % [N] tensión del cable de izaje con el spreader vacío suspendido (valor por debajo significa carga apoyada)
F_hw_susp_min = 0.5 * F_hw_min;  % [N] tensión mínima para considerar la carga suspendida (habilitación del control de balanceo; margen frente a las oscilaciones de F_hw con el spreader vacío)

% Parámetros de contacto para carga apoyada
% c: contact
K_cy = 1.8e9;       % [N/m] rigidez  por contacto vertical (compresión)
b_cy = 1.0e7;       % [N/(m/s)] fricción por contacto vertical (compresión)
b_cx = 1.0e6;       % [N/(m/s)] fricción por contacto horizontal (arrastre)

% Estado inicial de la carga
% l_h0 = 0;
% y_h0 = Y_t0 - l_h0;
% y_h0 = H_c.Value;
l_h0 = Y_t0 - y_h0;
x_l0 = x_t0;        % [m] posición horizontal inicial de la carga (igual a posición inicial del carro sin balanceo)
y_l0 = y_h0;        % [m] posición vertical inicial de la carga  (igual a altura de la carga sin balanceo)

% Frecuencia natural de balanceo de la carga
% w_n_theta = sqrt(g/l_h)
% Frecuencia más baja se da en caso l_h_max
l_h_max = Y_t0 - y_h_min_oper.Value;
w_n_theta_min = sqrt(g.Value/l_h_max);


%% CONTROLADORES DE MOVIMIENTO

% Controladores PI de velocidad: F* = K_pv*e_v + K_iv*integral(e_v), con
% e_v = v* - v. Sobre la planta equivalente 1/(m_eq*s + b_eq), la ecuación
% característica es m_eq*s^2 + (b_eq + K_pv)*s + K_iv = 0; igualando con
% m_eq*(s^2 + 2*xi*w*s + w^2) resulta K_pv = 2*xi*w*m_eq - b_eq y K_iv = w^2*m_eq.
% Anti-windup: el integrador se limita de modo que la acción I no supere la
% fuerza equivalente máxima del accionamiento (F_eq_max/K_iv).

% ==================== Control de traslación del carro ====================

% Movimiento de traslación del carro
% Función de transferencia: H_t(s)=V_t(s)/F_t_eq(s)=1/(m_t_eq*s+b_t_eq)
p_t = -b_t_eq.Value/m_t_eq;  % [1/s] polo del subsistema a lazo abierto (negativo: sistema estable)

% Sistema deseado a lazo cerrado de velocidad del carro (doc. 4: w = 10*|p|, xi = 0.7)
% Realimentación: velocidad del cable en el tambor v_td (encoder del motor,
% guía 3.2.a), colocada con la fuerza del motor. Con la velocidad del carro v_t
% el modo del cable de carro (~5.7 rad/s) se inestabiliza con estas ganancias
w_tc = 10 * abs(p_t);  % [1/s] frecuencia natural deseada del lazo cerrado de velocidad
xi_tc = 0.7;           % [-] factor de amortiguamiento deseado del lazo cerrado de velocidad

% Ganancias del controlador PI de velocidad del carro
K_pv_t = 2*xi_tc*w_tc*m_t_eq - b_t_eq.Value;  % [N/(m/s)] ganancia proporcional
K_iv_t = w_tc^2*m_t_eq;                       % [N/m] ganancia integral

% Anti-windup: fuerza equivalente máxima del accionamiento de carro
F_t_eq_max = T_tm_max * i_t / r_td;  % [N]


% ===================== Control de izaje de la carga ======================

% Movimiento de izaje (fuerza total de los dos ramales: perturbación 2*F_hw)
% Función de transferencia: H_h(s)=V_h(s)/F_h_eq(s)=1/(m_h_eq*s+b_h_eq), v_h = -l_h_dot
p_h = -b_h_eq/m_h_eq;  % [1/s] polo del subsistema a lazo abierto (negativo: sistema estable)

% Sistema deseado a lazo cerrado de velocidad de izaje (doc. 4: w = 10*|p|, xi = 0.7)
% Realimentación: v_h = -l_h_dot (encoder del motor de izaje, guía 3.2.a)
w_hc = 10 * abs(p_h);  % [1/s] frecuencia natural deseada del lazo cerrado de velocidad
xi_hc = 0.7;           % [-] factor de amortiguamiento deseado del lazo cerrado de velocidad

% Ganancias del controlador PI de velocidad de izaje (más compensación
% anticipativa de 2*F_hw en el modelo)
K_pv_h = 2*xi_hc*w_hc*m_h_eq - b_h_eq;  % [N/(m/s)] ganancia proporcional
K_iv_h = w_hc^2*m_h_eq;                 % [N/m] ganancia integral

% Anti-windup: fuerza equivalente máxima del accionamiento de izaje (dos
% ramales, a velocidad nominal del motor)
F_h_eq_max = 2 * T_hm_MAX * i_h / r_hd;  % [N]


% ==================== Control de balanceo de la carga ====================
% th: theta (relacionado a balanceo de la carga)

% Controlador de balanceo de la carga: PD alrededor del equilibrio dinámico
% (guía 4.2), sin acción integral (k_p3_th = 0 => K_i_th = 0): con integral y
% consigna theta = 0 el control cancelaba la aceleración pedida por el operador.
% Con w_c_th = w_n_th el control no cambia la rigidez del péndulo y equivale a
% inyectar amortiguamiento: a_theta ~ (2*xi*sqrt(g*l_h) - 2*l_h_dot)*theta_dot
% w_c_th = k_w_c_th * w_n_th
% w_c_th: frecuencia natural deseada del subsistema a lazo cerrado
% w_n_th: frecuencia natural del subsistema físico a lazo abierto
k_w_c_th = 1;   % [-] factor de frecuencia natural deseada del subsistema a lazo cerrado
% p3_th = k_p3_th * w_c_th
% p3_th: tercer polo del susbsistema a lazo cerrado (0: controlador PD)
k_p3_th = 0;    % [-] factor del tercer polo a lazo cerrado
xi_c_th = 1;    % [-] factor de amortiguamiento deseado (crítico, guía 4.2)
k_th = 1;     % [-] factor de ponderación de control de balanceo de carga sobre actuación del carro

% Corrección de la consigna de velocidad del carro por balanceo: la aceleración
% a_theta (linealización por realimentación) se satura en a_t_max y se integra
% con fuga: d(dv_t_th)/dt = a_theta - dv_t_th/tau_w_th
tau_w_th = 10;                % [s] constante de tiempo de la fuga (washout) de la corrección
dv_t_th_max = 0.25 * v_t_max; % [m/s] corrección máxima de la consigna de velocidad del carro

% Subida limitada con balanceo grande (función limit_v_h del supervisor):
% acortar el cable amplifica la amplitud del balanceo (~l^(-3/4)). Cerca de los
% fines de carrera del carro, el control de balanceo no tiene recorrido para
% corregir (simulación: con el carro detenido a 2 m o más del límite, el
% balanceo queda en ~12° sin limitar la subida; a 1 m o menos supera 20°)
d_sway_margin = 2;                       % [m] distancia al fin de carrera del carro considerada "cerca"
theta_amp_hoist_near = 5 * pi/180;       % [rad] amplitud de balanceo máxima para subir a velocidad normal cerca de los fines de carrera
theta_amp_hoist_far = 10 * pi/180;       % [rad] ídem en el resto del recorrido
v_h_sway_slow = 0.1 * v_h_nom;           % [m/s] velocidad de subida con balanceo mayor que el umbral

%% CONTROLADORES DE MOVIMIENTO

% =========================== Creación de buses ===========================
% Sensor bus
sensor_elem(1) = Simulink.BusElement;
sensor_elem(1).Name = 'x_t_dot';
sensor_elem(1).DataType = 'double';

sensor_elem(2) = Simulink.BusElement;
sensor_elem(2).Name = 'l_h';
sensor_elem(2).DataType = 'double';

sensor_elem(3) = Simulink.BusElement;
sensor_elem(3).Name = 'l_h_dot';
sensor_elem(3).DataType = 'double';

sensor_elem(4) = Simulink.BusElement;
sensor_elem(4).Name = 'theta_l';
sensor_elem(4).DataType = 'double';

sensor_elem(5) = Simulink.BusElement;
sensor_elem(5).Name = 'theta_l_dot';
sensor_elem(5).DataType = 'double';

sensor_elem(6) = Simulink.BusElement;
sensor_elem(6).Name = 'F_hw';
sensor_elem(6).DataType = 'double';

sensor_elem(7) = Simulink.BusElement;
sensor_elem(7).Name = 'x_t';
sensor_elem(7).DataType = 'double';

sensor_elem(8) = Simulink.BusElement;
sensor_elem(8).Name = 'T_tm';
sensor_elem(8).DataType = 'double';

% Fines de carrera de operación (entradas digitales)
limit_switch_names = {'ls_t_min', 'ls_t_max', 'ls_h_min', 'ls_h_max'};  % carro: x_t <= x_t_min_oper, x_t >= x_t_max_oper; izaje: y_h <= y_h_min_oper, y_h >= y_h_max_oper
for k = 1:numel(limit_switch_names)
    sensor_elem(8 + k) = Simulink.BusElement;
    sensor_elem(8 + k).Name = limit_switch_names{k};
    sensor_elem(8 + k).DataType = 'boolean';
end

command_bus = Simulink.Bus;
command_bus.Elements = sensor_elem;
assignin('base', 'sensor_bus', command_bus);

% Command bus
command_elem(1) = Simulink.BusElement;
command_elem(1).Name = 'translation';
command_elem(1).DataType = 'double';

command_elem(2) = Simulink.BusElement;
command_elem(2).Name = 'hoist';
command_elem(2).DataType = 'double';

command_elem(3) = Simulink.BusElement;
command_elem(3).Name = 'TLK';
command_elem(3).DataType = 'boolean';

command_bus = Simulink.Bus;
command_bus.Elements = command_elem;
assignin('base', 'command_bus', command_bus);

% =================== Parámetros de control supervisor ====================
l_h_tol = 1.0;        % [m] tolerancia llegada a l_h_peak
l_h_man_thresh = 1.0; % [m] umbral zona manual izaje
x_t_man_thresh = 1.0; % [m] umbral zona manual traslación

% Freno de operación del carro
v_t_brk = 0.05 * v_t_max;      % [m/s] velocidad de carro bajo la cual se cierra el freno (w_tm_MinBRK = i_t * v_t_brk / r_td)
theta_l_brk = 1.0 * pi / 180;  % [rad] balanceo bajo el cual se cierra el freno (ángulo residual)
T_tm_proof = 0.1 * T_tm_max;   % [N.m] torque de motor de carro que valida la apertura del freno (torque proving)





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Función de transferencia: H_t(s)=V_t(s)/F_t_eq(s)=1/(m_t_eq*s+b_t_eq)
% p_t = -b_t_eq/m_t_eq;     % [1/s] polo del subsistema (negativo: sistema estable)
% w_tn = -p_t;                % [1/s] frecuencia natural del subsistema a lazo abierto
% 
% %% Borrar [N/(m/s)] / kg = N * s / (m * kg), N = kg * m / s^2 => kg * m / s = N * s
% %% N * s / (m * kg) = N * (1/(N*s)) = 1/s


% % Método de sintonía serie
% % tau_hm = 1.0;           % [ms] constante de tiempo de modulador de torque en motor-drive de izaje
% 
% 
% 
% w_pos_h = 5 * w_h;      % [] frecuencia del controlador
% n_h = 2.5;
% ba_h = J_eqh * n_h * w_pos_h;
% Ksa_h = J_eqh * n_h * w_pos_h^2;
% Ksia_h = J_eqh * w_pos_h^3;
% 
% 
% 
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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