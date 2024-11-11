% =========================================================================
% LABORATORIO DE CONTROL DE ALTURA DEL DRON CRAZYFLIE
% Sistemas de control 1
% -------------------------------------------------------------------------
% Parte 3: Experimentación física de controlador PID de altura para el dron
% Crazyflie 2.1
% =========================================================================

%% Descripción
% En este archvio se presenta el algoritmo para modificar el controlador
% PID de altura del dron Crazyflie junto a los comandos para captura de
% datos empleando el sistema de captura de movimiento del Robotat.

%% Parte 1: Carpeta con herramientas de software 
% Se añade al path de Matlab la carpeta con las funciones Crazyflie
addpath('crazyflie');
% Se añade al path de Matlab la carpeta con las funciones Robotat
addpath('robotat');

%% Parte 2: Conectar al sistema Robotat
robotat = robotat_connect(); % Conexión inicial con el sistema Robotat
marker_id = 50; % ID del marcador asignado al dron Crazyflie

%% Parte 3: Configuración del Timer para Captura de Datos
% Crear variables globales para almacenar datos
global pos_data time_data timer_flag;
pos_data = [];    % Almacenar posición [x, y, z]
time_data = [];   % Almacenar tiempo de captura
timer_flag = true; % Flag para controlar el timer

% Crear el objeto timer
data_timer = timer('ExecutionMode', 'fixedRate', ...
                   'Period', 0.1, ...             % Intervalo de tiempo (0.1s)
                   'TimerFcn', {@capture_pose, robotat, marker_id}); % Función callback

%% Parte 4: Establecer valores para las ganancias del controlador
% Valores a establecer en las ganacias del controlador PID del eje Z
Kp = 2.50; % Ganancia propocional
Ki = 1.00; % Ganancia integrativa
Kd = 0.001; % Ganancia derivativa

%% Parte 5: Secuencia del experimento
% RECORDATORIO: Previo a ejecutar esta sección debe conectar el dispositivo 
% Crazyradio en algún puerto USB de su ordenador y encender el Crazyflie. 
% Importante, antes de ejecutar esta sección de código debe asegurarse que
% el dron tiene espacio físico suficiente para completar el vuelo.

% Inicializar cronómetro para registrar el tiempo de vuelo
tic;

% Secuencia del experimento:
% Conexión con Crazyflie
dron_id = 8; % ID del dron Crazyflie específico a utilizar
crazyflie_1 = crazyflie_connect(dron_id);
% Configurar las nuevas ganancias en el controlador PID del eje Z
crazyflie_set_pid_z(crazyflie_1, Kp, Ki, Kd);

% Iniciar el Timer antes del despegue
start(data_timer);

% Despegue a altura predeterminada
altura_objetivo = 0.5; % en metros
crazyflie_takeoff(crazyflie_1, altura_objetivo); 
% Tiempo de vuelo luego del despegue
pause(8); % Colocar un tiempo considerable para que se estabilice 
% Aterrizaje
crazyflie_land(crazyflie_1); 

% Finalizar la captura de datos (detener el timer)
timer_flag = false;
stop(data_timer); % Detener el timer inmediatamente

% Desconectar del dron y Robotat
crazyflie_disconnect(crazyflie_1);
robotat_disconnect(robotat);

delete(data_timer);

%% Parte 6: Graficar los datos obtenidos
% Graficar resultados
figure;
%subplot(2, 1, 1);
plot(time_data, pos_data(:, 3), 'LineWidth', 2);
hold on
yline(altura_objetivo, '--', 'Altura objetivo', 'LineWidth', 1.5, 'Color', [0.5, 0, 0]); % Línea punteada en 0.5
hold off
xlabel('Tiempo (s)');
ylabel('Altura (m)');
title('Experimentación de controlador PID de altura');
grid on;

%%
error_data = altura_objetivo - pos_data(:, 3);
subplot(2, 1, 2);
plot(time_data, error_data, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Error (m)');
title('Error de Altura');
grid on;