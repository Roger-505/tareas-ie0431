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
Kt = 100/3.25;      % ambito maximo de medicion del sensor es de 3.25m

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
K1 = (2 / (Xvs0 * Kvs)) * sqrt(H0 /(rho * g));
K2 = -2 * H0/Xvs0;
T  = (2 * A / (Xvs0 * Kvs)) * sqrt(H0 /(rho * g));

%% f)i) Simulación
tu=4000;                %tiempo simulacion
sim('db.slx',tu)        %comando para simular el DB

Hlineal = ans.y1;
Hreal = ans.y2;

figure (1)
x1=xlabel ('$t$ [s]');
y1=ylabel ('$H$ [\%]');
hold on;
plot(Hlineal, ...
    'LineWidth', 2, ...
    'Color', 'r')
plot(Hreal, ...
    'LineWidth', 2, ...
    'Color', 'b')
grid on;
leg1 = legend('$H _{linealizado}(t)$', '$H _{real} (t)$');
set(y1,'Interpreter','latex');
set(y1,'FontSize',12);
set(x1,'Interpreter','latex');
set(x1,'FontSize',12);
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',12);
set(leg1, 'Location','northwest');
hold off;


%% f)ii) Cambios de -10% en U y -0.1 en la perturbación 

deltau = u0 - 10;       % Cambio de -10% en U
steptimed = 1500;       % la perturbación se aplica hasta que el sistema se estacione del cambio en Qe
deltad = Xvs0 - 0.1;    % Cambio -0.1 en la perturbación

tu=4000;                %tiempo simulacion
sim('db.slx',tu)        %comando para simular el DB

Hlineal = ans.y1;
Hreal = ans.y2;

figure (2)
x1=xlabel ('$t$ [s]');
y1=ylabel ('$H$ [\%]');
hold on;
plot(Hlineal, ...
    'LineWidth', 2, ...
    'Color', 'r')
plot(Hreal, ...
    'LineWidth', 2, ...
    'Color', 'b')
grid on;
leg1 = legend('$H _{linealizado}(t)$', '$H _{real} (t)$');
set(y1,'Interpreter','latex');
set(y1,'FontSize',12);
set(x1,'Interpreter','latex');
set(x1,'FontSize',12);
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',12);
set(leg1, 'Location','northwest');
hold off;

%% g) ecuacion caracteristica c(t) vs m(t)
Qeperc =linspace(0,100,101)';
Qe = Kvc.* Qeperc;
Hlow  = (1/(rho*g)) .* (Qe./ (Xvsmin .* Kvs)).^2 .*Kt;
Hpo   = (1/(rho*g)) .* (Qe./ (Xvs0 .* Kvs)).^2 .*Kt;
Hhigh = (1/(rho*g)) .* (Qe./ (Xvsmax .* Kvs)).^2 .*Kt;

figure (3)
x1=xlabel ('$Q _e$ [\%]');
y1=ylabel ('$H$ [\%]');
hold on;
plot(Qeperc,Hlow, ...
    'LineWidth', 2, ...
    'Color', 'r');
plot(Qeperc,Hpo, ...
    'LineWidth', 2, ...
    'Color', 'b');
plot(Qeperc,Hhigh, ...
    'LineWidth', 2, ...
    'Color', 'g');
stem(Qe0/Kvc, H0*Kt);
leg1 = legend('$X _{vs} = 0.4$', '$X _{vs} = 0.5$', '$X _{vs} = 0.6$', 'Punto de operacion');
set(y1,'Interpreter','latex');
set(y1,'FontSize',12);
set(x1,'Interpreter','latex');
set(x1,'FontSize',12);
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',12);
set(leg1, 'Location','northwest');
grid on;
hold off;

%% funciones
% caudal a partir de la caracteristica estatica
function Qe = cEstatica(Xvs, Kvs, rho, g, H)
    Qe = Kvs * Xvs* (rho * g * H)^(1/2); 
end

