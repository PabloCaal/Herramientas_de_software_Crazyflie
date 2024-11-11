% =========================================================================
%                     ROBÓTICA - LABORATORIO: 
% Generación y seguimiento de trayectorias con drones Crazyflie 2.1
% -------------------------------------------------------------------------
% Puede ver el detalle de los pasos en la guía adjunta, pero se le deja un
% resumen:
% 0. Reciba las instrucciones generales al inicio del laboratorio.
% 1. Seleccione un dron Crazyflie a utilizar. Este y el siguiente 
% laboratorio se estarán trabajando en PAREJAS.
% 2. Verifique que puede conectarse al dron de forma inalámbrica y que 
% puede obtener correctamente su pose dentro del Robotat.
% 3. Utilice la información del sistema de captura para definir, en donde
% se indica, los obstáculos y puntos relevantes (despegue y aterrizaje).
% Realice un ajuste de orientación de los obstáculos respecto de las
% orientaciones originales de sus markers. 
% Con ello, genere trayectorias evaluando los distintos métodos de
% interpolación dispoibles y verifique que la visualización de la 
% trayectoria coincide con lo que observa sobre la plataforma del Robotat.
% 4. Experimente y familiaricese con el dron Crazyflie empleando las 
% funciones de control de alto nivel.
% 5. Combine el resultado de la generación de trayectorias con las 
% funciones de control del dron Crazyflie para ejecutar físicamente el 
% seguimiento de la trayectoria.
% 6. Verifique que la generación y ejecución continúa funcionando
% adecuadamente variando la posición del punto de aterrizaje y la 
% distribución física de los obstáculos.
% =========================================================================

%% Carpetas con herramientas de software
% Se añade al path de Matlab la carpeta con las funciones Crazyflie
addpath('crazyflie');
% Se añade al path de Matlab la carpeta con las funciones Robotat
addpath('robotat');

%% Conexión al Robotat 
robotat = robotat_connect();

%% Capturar información de la pista de obstáculos
% En esta sección debe obtener la pose de los markers relevantes de la
% pista de obsáculos para posteriormente generar la ruta de obstáculos

% Información de los obstáculos
% En ese ejemplo se utilizaron 3 obstáculos con los markers 11, 15 y 13
% Para definir el punto de despegue se utilizó el marker 12
% Para definir el punto de aterrizaje se utilizó el marker 16

obstacle_id = [11, 15, 13]; % Arreglo con id de los markers de obstáculos
N = 3; % Cambia este valor según el número de obstáculos

% Capturar la información de pose de los markers 
% Punto de despegue
punto_inicial = robotat_get_pose(robotat, 12, "eulxyz");
% Punto de aterrizaje
punto_final = robotat_get_pose(robotat, 16, "eulxyz");
% Osbtáculos
obstaculos = zeros(N, 6); % Crear una matriz para almacenar la pose de N obstáculos
% Captura la pose de los obstáculos de forma dinámica
for i = 1:N
    obstaculos(i,:) = robotat_get_pose(robotat, obstacle_id(i), "eulxyz"); % IDs secuenciales
end

robotat_disconnect(robotat); % Cerrar conexión

%% Ajuste de orientación de obstáculos
% La orientación obtenida de los markers 11, 15 y 13 no es la misma
% orientación que poseen los obstáculos. Por esta razón, es necesario
% ajustar la orientación de la pose de estos markers y así poder generar
% correctamente la trayectoria a volar.

% Para determinar el valor de offset de cada obstáculo es necesario colocar
% al obstáculo en distintas poses conocidas (como 0°, 90°, 180°, etc.)
% respécto del sistema de coordenadas del Robotat y así poder determinar el
% desfase que posee la orientación del marker respecto de la orientación
% del obstáculo. 

% Puede tomar un marker de los del ejemplo (11, 15 o 13) y tomar su pose en
% las poses conocidas para observar el origen del valor de desfase colocado
% en está sección:

% Offset para marker 11 en obstáculo
offset_marker11 = -201;
obstaculos(1, 6) = mod(obstaculos(1, 6), 360);
obstaculos(1, 6) = obstaculos(1, 6) + offset_marker11;
if obstaculos(1, 6) < 0
    obstaculos(1, 6) = obstaculos(1, 6) + 360;
end

% Offset para marker 15 en obstáculo
offset_marker15 = 315;
obstaculos(2, 6) = mod(obstaculos(2, 6), 360);
obstaculos(2, 6) = obstaculos(2, 6) + offset_marker15;
if obstaculos(2, 6) >= 360
    obstaculos(2, 6) = obstaculos(2, 6) - 360;
end

% Offset para marker 13 en osbtáculo
offset_marker15 = - 318;
obstaculos(3, 6) = mod(obstaculos(3, 6), 360);
obstaculos(3, 6) = obstaculos(3, 6) + offset_marker15;
if obstaculos(3, 6) < 0
    obstaculos(3, 6) = obstaculos(3, 6) + 360;
end

%% GENERACIÓN DE TRAYECTORIA
% Utilizando la información de pose extraída de los markers colocados en la
% pista de obstáculos genere otros puntos clave que le puedan ser de ayuda
% para generar la trayectoria.

% Para realizar la interpolación de puntos, utilizará el punto al que se
% elevará el dron al inicio, puntos antes y después de cada obstáculo y el
% punto al que llegará previo a aterrizar en el punto final.

% PUNTO DE DESPEGUE
% Defina una altura a la que se elevará inicialmente el dron previo a
% ejecutar la trayectoria (este será el primer punto de la trayectoria)
altura_de_despegue = 0.3;
punto_de_despegue = punto_inicial(1:3) + [0, 0, altura_de_despegue];

% PUNTO DE ATERRIZAJE
% Defina una altura a la que el dron llegará previo a ejecutar el
% aterrizaje (este será el último punto de la trayectoria)
altura_de_aterrizaje = 0.2;
punto_de_aterrizaje = punto_final(1:3) + [0, 0, altura_de_aterrizaje];

% PUNTOS CLAVE PARA ATRAVESAR LOS OBSTÁCULOS
% Geometría del obstáculo
diameter = 0.32;
radius = diameter / 2;
theta = linspace(0, 2*pi, 7); % Generar el hexágono
% Para tener seguridad de que el dron no colisionará con los obstáculos, se
% determina una distancia de 0.1 metros pre y post centro del obstáculo:
obstacle_distance = 0.1; % Distancia pre y post obstáculos

% Arreglos para almacenar puntos importantes
obstacle_center = zeros(N, 6); % Puntos de centro de obstáculos
pre_obstacle = zeros(N, 3); % Puntos pre-obstáculos
post_obstacle = zeros(N, 3); % Puntos post-obstáculos

% Función para aplicar la orientación adecuada
apply_yaw = @(x, y, yaw) [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)] * [x; y];

% Ciclo para determinar puntos pre, central y post obstáculos
for i = 1:size(obstaculos,1)
    % Determinar el conjunto de los puntos de centro de los obstáculos
    obstacle_center(i,1:3) = obstaculos(i,1:3) - [0, 0, radius];
    % Ajustar la orientación de los obstáculos con el offset indicado
    yaw_obstacle = deg2rad(obstaculos(i,6));  
    hexagon_y = radius * cos(theta); 
    hexagon_z = radius * sin(theta) + obstacle_center(i,3);
    rotated_hexagon = apply_yaw(zeros(size(hexagon_y)), hexagon_y, yaw_obstacle);
    % Determinar los puntos pre y post centro de los obstáculos
    pre_obstacle(i,:) = obstacle_center(i,1:3) - obstacle_distance*[cos(yaw_obstacle), sin(yaw_obstacle), 0];
    post_obstacle(i,:) = obstacle_center(i,1:3) + obstacle_distance*[cos(yaw_obstacle), sin(yaw_obstacle), 0];
end

% Generación de los puntos de la trayectoria a través de los obstáculos
trajectory_points = punto_de_despegue;
% Añadir puntos pre, centro y post de cada obstáculo
for i = 1:N
    trajectory_points = [trajectory_points; pre_obstacle(i,:); post_obstacle(i,:)];
end
% Finalmente añadimos el punto de aterrizaje
trajectory_points = [trajectory_points; punto_de_aterrizaje];

% INTERPOLACIÓN DE LA TRAYECTORIA
% Utilizando los puntos clave definidos con la informaciónd de los markers,
% utilice distintos métodos de interpolación para generar una trayectoria
% SUAVE para que el dron Crazyflie no tenga dificultades de realizarla.

n_interp = 30; % Número de puntos interpolados
t = 1:size(trajectory_points, 1); % Tamaño de la trayectoria
t_interp = linspace(1, t(end), n_interp); % Cantidad de muestras para iteración

% Interpolación
% Pruebe los distintos métodos de interpolación que ofrece el comando
% interp1. Presiono F1 sobre el comando para averiguar qué métodos de
% interpolación existen.
x = interp1(t, trajectory_points(:, 1), t_interp, 'spline');
y = interp1(t, trajectory_points(:, 2), t_interp, 'spline');
z = interp1(t, trajectory_points(:, 3), t_interp, 'spline');

%% VISUALIZACIÓN DE TRAYECTORIA EN RUTA CON OBSTÁCULOS
% Graficar los puntos relevantes de la trayectoria en un espacio 3D
figure;
hold on;
[x_floor, y_floor] = meshgrid(-2:0.1:2, -2.5:0.1:2.5);
z_floor = zeros(size(x_floor));
surf(x_floor, y_floor, z_floor, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5); % Dibujar el piso
% Graficar los puntos: inicio, despegue, aterrizaje y final
plot3(punto_inicial(1), punto_inicial(2), punto_inicial(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
% text(initial_point(1), initial_point(2), initial_point(3), 'Origin', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
plot3(punto_de_despegue(1), punto_de_despegue(2), punto_de_despegue(3), 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
% text(takeoff_point(1), takeoff_point(2), takeoff_point(3), 'Takeoff', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
plot3(punto_de_aterrizaje(1), punto_de_aterrizaje(2), punto_de_aterrizaje(3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
% text(land_point(1), land_point(2), land_point(3), 'Land Point', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
plot3(punto_final(1), punto_final(2), punto_final(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% text(final_point(1), final_point(2), final_point(3), 'Final Point', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Graficar los obstáculos en el espacio 3D
for i = 1:size(obstaculos,1)
    plot3(obstaculos(i,1)+rotated_hexagon(1,:), obstaculos(i,2)+rotated_hexagon(2,:), hexagon_z, 'r', 'LineWidth', 2);    
end

% Graficar la trayectoria interpolada
plot3(x, y, z, 'm-', 'LineWidth', 2);
plot3(x, y, z, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Puntos interpolados

% Configurar los ejes
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Visualización de la trayectoria generada');
grid on;
axis equal;
xlim([0.3 1.5]); % Ajustar límites en X
ylim([-1.2 1.2]); % Ajustar límites en Y
zlim([0 0.5]); % Ajustar límites en Z para ver desde el suelo hacia arriba
view(3);
hold off;

%% SEGUIMIENTO DE TRAYECTORIA CON FUNCIONES CRAZYFLIE DE ALTO NIVEL 
% IMPORTANTE LEER:
% Una vez logre visualizar una trayectoria suave que pase correctamente por
% todos los obstáculos de la trayectoria, proceda a ejecutar el vuelo con
% el dron Crazyflie.

% Recordatorio: Previo a ejecutar esta sección debe conectar el dispositivo 
% Crazyradio en algún puerto USB de su ordenador y encender el Crazyflie. 

% Importante, antes de ejecutar esta sección de código debe asegurarse que
% el dron tiene espacio físico suficiente para completar la trayectoria.
% Ningún objeto debe interferir el espacio por el que se desplazará el dron

% Sustituya al marker en el punto de inicio (marker 12) por el dron
% Crazyflie. Asegúrese de que la orientación de los ejes del dron sea
% exactamente la misma que la del sistema Robotat. Es decir, los ejes XYZ 
% del dron deben coincidir con los XYZ del sistema Robotat.

% Si tiene duda de los ejes de cada sistema consulte las imágenes en el
% anexo de la guía de laboratorio.

%% Secuencia del experimento:
% Luego de colocar al dron en la posición indicada con la orientación
% correcta puede ejecutar los comandos para seguir la trayectoria:

% Conexión con Crazyflie
% Colocar el número de dron y el número de marker del dron:
% Si no sabe uno de estos datos pregunte al catedrático
agent_id = 50; % ID del marker del dron
dron_id = 8; % Número de dron 
crazyflie_1 = crazyflie_connect(dron_id); 

% Despegue a una altura específica en un tiempo dado
tiempo_de_despegue = 0.5;
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

%% Desconexión del Crazyflie 2.1
% Desconexión
crazyflie_disconnect(crazyflie_1); 

%% Desconexión del Robotat
robotat_disconnect(robotat);