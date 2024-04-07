% Tarea #1: Sistemas de Control IE0431
% Estudiante: Roger Daniel Piovet Garcia 
% Carnet: C15990

% El codigo mostrado a continuacion se encuentra en el
% siguiente repositorio de Github:
%
% https://github.com/Roger-505/tareas-ie0431

% inicio del script
close all
clc

%% PARTE A

% Declaracion de vector de tiempo y de la variable
% del dominio de Laplace
t = 0:0.01:10;
s = tf('s');

% Declaracion de parametros 
Kp = 1;
K = 1;
L = 0;
T = 0.2;

% Proceso integrante de segundo orden y
% Controlador proporcional
P = K * exp(-L*s)/(1+T*s);
C = Kp;

% Sistema como servocontrol y regulador
% Respuesta del sistema
MydA = minreal(P/(1+ C * P));
MyrA = minreal(MydA * C);

% Respuesta del controlador
MurA = minreal(C/(1+ C * P));
MudA = minreal(MydA * -P);

% Referencia unitaria a partir de t = 1,
% perturbacion unitaria a partir de t = 6
r = 0;
r ( t >= 1) = 1;
d = 0; 
d ( t >= 6) = 1;

% Simulacion de servocontrol y regulador
% Respuesta del sistema
yrA = lsim(MyrA, r ,t);
ydA = lsim(MydA ,d ,t );
yA = yrA + ydA;

% Respuesta del controlador
urA = lsim(MurA, r ,t);
udA = lsim(MudA ,d ,t );
uA = urA + udA;

% Grafica de respuesta del sistema
figure (1) ;
title ('Respuesta del sistema como Servocontrol y como Regulador');
xlabel ('Tiempo (s)');
ylabel ('Respuesta del sistema');
hold on;
plot (t , yA, ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'r');
grid on;
legend('y(t)', 'd(t)', 'r(t)')
hold off;

% Grafica de respuesta del controlador
figure (2) ;
title ('Respuesta del Controlador como Servomecanismo y como Regulador');
xlabel ('Tiempo (s)');
ylabel ('Respuesta del controlador');
hold on;
plot (t , uA, ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'r');
grid on;
legend('u(t)', 'd(t)', 'r(t)')
hold off;

%% PARTE B
G

