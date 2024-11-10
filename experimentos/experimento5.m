% =========================================================================
% EXPERIMENTOS CON CRAZYFLIE 2.1
% -------------------------------------------------------------------------
% Experimento 5: Trayectoria lineal con fusión de sensores
% =========================================================================

%% Parte 1: Carpetas con herramientas de software
% Se añade al path de Matlab la carpeta con las funciones Crazyflie
addpath('crazyflie');
% Se añade al path de Matlab la carpeta con las funciones Robotat
addpath('robotat');

%% Parte 2: Conexión con Robotat y Crazyflie
dron_id = 8; % ID del dron Crazyflie a utilizar
marker_id = 50; % ID del marker del Crazyflie dentro del Robotat

% Conexión con Robotat y Crazyflie
robotat = robotat_connect();
crazyflie_1 = crazyflie_connect(dron_id);

%% Parte 3: Generación de trayectoria
% Se generará una trayectoria lineal simple de 1.0 metro de distancia a
% partir del punto de despegue en dirección del eje X del Crazyflie. 

% Obtener la pose inicial del dron por medio sistema de captura del Robotat
posicion_inicial = robotat_get_pose(robotat, marker_id, 'eulxyz');
% Establecer la posición inicial absoluta del dron
crazyflie_set_position(crazyflie_1, posicion_inicial(1), posicion_inicial(2), posicion_inicial(3));

% Información del vuelo 
altura_de_despegue = 0.50; % (en metros)
tiempo_de_despegue = 0.75; % (en segundos)
distancia_de_vuelo = 1.00; % (en metros)
velocidad_de_vuelo = 1.00; % (en metros/segundo)

% Generación de puntos clave de la trayectoria
punto_inicial_de_trayectoria = posicion_inicial(1:3) + [0.0, 0.0, altura_de_despegue];
punto_final_de_trayectoria = punto_inicial_de_trayectoria + [distancia_de_vuelo, 0.0, 0.0];
punto_de_aterrizaje = punto_final_de_trayectoria - [0.0, 0.0, altura_de_despegue];

% Generación de la trayectoria
N = 5; % Cantidad de puntos en la trayectoria
x = linspace(punto_inicial_de_trayectoria(1), punto_final_de_trayectoria(1), N);
y = linspace(punto_inicial_de_trayectoria(2), punto_final_de_trayectoria(2), N);
z = linspace(punto_inicial_de_trayectoria(3), punto_final_de_trayectoria(3), N);

% Visualización 3D
figure;
plot3(x, y, z, 'r*-', 'DisplayName', 'Trayectoria'); % Línea de la trayectoria
hold on;
plot3(posicion_inicial(1), posicion_inicial(2), posicion_inicial(3), 'go', 'MarkerSize', 10, 'DisplayName', 'Posición incial'); % Punto de despegue en verde
plot3(punto_de_aterrizaje(1), punto_de_aterrizaje(2), punto_de_aterrizaje(3), 'bo', 'MarkerSize', 10, 'DisplayName', 'Punto de Aterrizaje'); % Punto de aterrizaje en azul
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Trayectoria generada');
legend; % Muestra la leyenda
grid on;
axis equal;
% Ajustar dimensiones dependiendo del cuadrante del Robotat en el que se
% encuentre realizando el experimento: 
axis([-2 2 -2.5 2.5 0 2]); 
view(3);
hold off;

%% Parte 4: Ejecución de vuelo en Crazyflie con Flow Deck
% Recordatorio: Previo a ejecutar esta sección debe conectar el dispositivo 
% Crazyradio en algún puerto USB de su ordenador y encender el Crazyflie. 

% Importante, antes de ejecutar esta sección de código debe asegurarse que
% el dron tiene espacio físico suficiente para completar la trayectoria.

% Secuencia del experimento:
% Despegue a una altura específica en un tiempo dado
crazyflie_takeoff(crazyflie_1, altura_de_despegue, tiempo_de_despegue); 
pause(0.5); % tiempo de seguridad
% Seguimiento de la trayectoria a la velocidad especificada con correción
% de posición utilizando lecturas del sistema de captura del Robotat
for i = 1:N
    crazyflie_goto_robotat(crazyflie_1, x(i), y(i), z(i), velocidad_de_vuelo, robotat, agent_id);
end
pause(0.5); % tiempo de seguridad
% Aterrizaje
crazyflie_land(crazyflie_1); 
% Desconexión
crazyflie_disconnect(crazyflie_1); 