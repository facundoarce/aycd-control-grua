% CONTROL SEMI-AUTOM�TICO COORDINADO DE GR�A PORTUARIA DE MUELLE
clc;
clear all;

% Par�metros del modelo
m_c = 50000.0;  % [kg] masa del carro (incluye sistema de izaje)

%% Traslaci�n del carro (eje x, horizontal, x=0 en borde del muelle)

%Par�metros traslaci�n:
R_w = 0.5;      % [m] radio primitivo de rueda
J_w = 2.0;      % [kg.m^2] momento de inercia de ruedas (en eje lento)
i_c = 15.0;     % [adimensional] relaci�n de reducci�n de caja reductora
J_m_c = 10.0;   % [kg.m^2] momento de inercia de motor y freno (en eje r�pido)
b_eq_c = 30.0;  % [N.m/(rad/s)] fricci�n m�canica equivalente

%Variables traslaci�n:
x_0 = -20.0;    % [m] posici�n inicial
x_min = -30.0;  % [m] posici�n m�nima (sobre muelle)
x_max = 50.0;   % [m] posici�n m�xima (sobre barco)
x_d_max = 4.0;  % [m/s] velocidad m�xima (cargado o sin carga)
x_dd_max = 1.0; % [m/s^2] aceleraci�n m�xima (cargado o sin carga)

m_eq_c = m_c + ( J_w + i_c^2*J_m_c ) / R_w^2;

%% Izaje de carga (eje y, vertical, y=0 al nivel del muelle)

%Par�metros izaje:
y_min = -20.0;  % [m] posici�n m�nima (dentro de barco)
y_max = 40.0;   % [m] posici�n m�xima (sobre barco)
y_t0 = 45.0;    % [m] altura (fija) de carro y sistema de izaje
y_sb = 15.0;    % [m] despeje m�nimo sobre borde de muelle (sill beam)
y_d_maxn = 1.5; % [m/s] velocidad m�xima (cargado con carga nominal)
y_d_max0 = 3.0; % [m/s] velocidad m�xima (sin carga)
y_dd_max = 1.0; % [m/s^2] aceleraci�n m�xima (cargado o sin carga)
Rd = 0.75;      % [m] di�metro de la rueda del tambor
J_d = 8.0;      % [kg.m^2] momento de inercia del tambor de izaje
ih = 30.0;      % [] relaci�n de transmisi�n de motor de izaje a tambor
J_mh = 30.0;    % [kg.m^2] momento de inercia del motor de izaje
b_eqh = 18.0;   % [N.m.s/rad] amortiguamiento viscoso de motor de izaje
%bd = ?
%bmh = ?

%Variables izaje:
J_eqi = J_d + J_mh * ih^2;
J_eqh = J_eqi / Rd^2;
% b_eqi=bd+bmh*ih^2;
% b_eqh=b_eqi/Rd^2;







%% ...