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
x1=xlabel ('$t$ [s]');
hold on;
plot (t , yA, ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'k');
grid on;
leg1=legend('$y(t)$', '$d(t)$', '$r(t)$');
set(x1,'Interpreter','latex');
set(x1,'FontSize',12);
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',12);
hold off;

% Grafica de respuesta del controlador
figure (2) ;
x2=xlabel ('$t$ [s]');
hold on;
plot (t , uA, ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'k');
grid on;
leg2=legend('$u(t)$', '$d(t)$', '$r(t)$');
set(x2,'Interpreter','latex');
set(x2,'FontSize',12);
set(leg2,'Interpreter','latex');
set(leg2,'FontSize',12);
hold off;

%% PARTE B
% Declaracion de parametros
Kp = 2;
Ti = 2;

% Controlador integrante
C = Kp*(1+ 1/(Ti * s));

% Sistema como servocontrol y regulador
% Respuesta del sistema
MydB = minreal(P/(1+ C * P));
MyrB = minreal(MydB * C);

% Respuesta del controlador
MurB = minreal(C/(1+ C * P));
MudB = minreal(MydB * -P);

% Referencia unitaria a partir de t = 1,
% perturbacion unitaria a partir de t = 6
r = 0;
r ( t >= 1) = 1;
d = 0; 
d ( t >= 6) = 1;

% Simulacion de servocontrol y regulador
% Respuesta del sistema
yrB = lsim(MyrB, r ,t);
ydB = lsim(MydB ,d ,t );
yB = yrB + ydB;

% Respuesta del controlador
urB = lsim(MurB, r ,t);
udB = lsim(MudB ,d ,t );
uB = urA + udB;

% Grafica de respuesta del sistema
figure (3) ;
x3=xlabel ('$t$ [s]');
hold on;
plot (t , yB, ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'k');
grid on;
leg3=legend('$y(t)$', '$d(t)$', '$r(t)$');
set(x3,'Interpreter','latex');
set(x3,'FontSize',12);
set(leg3,'Interpreter','latex');
set(leg3,'FontSize',12);
hold off;

% Grafica de respuesta del controlador
figure (4) ;
x4=xlabel ('$t$ [s]');
hold on;
plot (t , uB, ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'k');
grid on;
leg4=legend('$u(t)$', '$d(t)$', '$r(t)$');
set(x4,'Interpreter','latex');
set(x4,'FontSize',12);
set(leg4,'Interpreter','latex');
set(leg4,'FontSize',12);
hold off;

%% PARTE C
% Grafica de comparacion de las partes A y B
% Respuesta del sistema
figure (5) ;
x5=xlabel ('$t$ [s]');
hold on;
plot (t , yA, ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , yB, ...
    'LineWidth', 2.5, ...
    'Color', 'c');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'k');
grid on;
leg5=legend('$y_A(t)$', '$y_B(t)$', '$d(t)$','$r(t)$');
set(x5,'Interpreter','latex');
set(x5,'FontSize',12);
set(leg5,'Interpreter','latex');
set(leg5,'FontSize',12);
hold off;

% Respuesta del controlador
figure (6) ;
x6=xlabel ('$t$ [s]');
hold on;
plot (t , uA, ...
    'LineWidth', 2.5, ...
    'Color', 'b');
plot (t , uB, ...
    'LineWidth', 2.5, ...
    'Color', 'c');
plot (t , d, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'r');
plot (t , r, '--', ...
    'LineWidth', 2.5, ...
    'Color', 'k');
grid on;
leg6=legend('$u_A(t)$', '$u_B(t)$', '$d(t)$','$r(t)$');
set(x6,'Interpreter','latex');
set(x6,'FontSize',12);
set(leg6,'Interpreter','latex');
set(leg6,'FontSize',12);
hold off;

% Mapas de polos y ceros de Myr y Myd de cada sistema de control. 
% Parte A
figure (7);
pzplot(MyrA, 'rO');

figure (8);
pzplot(MydA, 'rO');

pzplot(MydA, 'rO');

% Parte B
figure (9);
pzplot(MyrB, 'rO');

figure (10);
pzplot(MydB, 'rO');
