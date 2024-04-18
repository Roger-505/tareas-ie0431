%% IE0431 Sistemas de Control
% Tarea 2 - Proceso Controlado
% Universidad de Costa Rica, Escuela de Ingeniería Eléctrica
% Roger Daniel Piovet Garcia, C15990, grupo 02
clc
clear all
close all

%% Parámetros del tanque y proceso
A = 5;          % área del tanque
g = 9.81;       % gravedad m/s2
Kvs = 0.001;    % constante de la válvula de salida del tanque
rho = 1027;     % densidad del líquido kg/m3 (agua de mar)

% valores de interes de la apertura de la valvula
Xvsmin = 0.4;   % minimo
Xvs0 = 0.5;     % deseado
Xvsmax = 0.6;   % maximo

% valores de interes del nivel del liquido en el tanque
Hmin = 2.24;    % minimo
H0 = 2.5;       % deseado
Hmax = 2.95;    % maximo

%% b) calculo de la ganancia del transmisor Kt (%/m)
Kt = 3.25/100;      % ambito maximo de medicion del sensor es de 3.25m

%% c) calculo de la ganancia de la valvula de control Kvc (m3/(%*s))
Qemax = cEstatica(Xvsmax, Kvs, rho, g, Hmax);  % caudal maximo a partir de la caracteristica estatica
Kvc = Qemax/100;        

%% d) Diagrama de bloques
steptimeu = 100;                        % el primer cambio se hace en 100 s
Qe0 = cEstatica(Xvs0, Kvs, rho, g, H0); % Se calcula con la ecuación de la característica estática y H0, Xvs0
u0 = Qe0/Kvc;                           % Punto operación del caudal de entrada en %
deltau = u0 - 2;                        % Cambio de -2% en U
steptimed = 1500;                       % la perturbación se aplica hasta que el sistema se estacione del cambio en Qe
deltad = Xvs0 - 0.02;                   % Cambio -0.02 en la perturbación

%% e)
% K1=
% K2=
% T=
% Kd=
% K=

%% f)i) Simulación
tu=3000; %tiempo simulacion
sim('cambiarnombrearchivosimulink',tu) %comando para simular el DB
plot(y3); % salida sistema real
hold on %graficar en una misma figura
% plot(ylineal)
%ylabel
%xlabel
% title
% legend

%% f)ii) Cambios de -10% en U y -0.1 en la perturbación 


%% funciones
% caudal en caracteristica estatica
function Qe = cEstatica(Xvs, Kvs, rho, g, H)
    Qe = Kvs * Xvs* (rho * g * H)^(1/2); 
end