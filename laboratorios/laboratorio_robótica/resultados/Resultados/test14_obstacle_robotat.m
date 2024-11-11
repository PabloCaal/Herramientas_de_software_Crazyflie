% =========================================================================
% PROYECTO DE GRADUACIÓN: HERRAMIENTAS DE SOFTWARE PARA CRAZYFLIE
% Pablo Javier Caal Leiva - 20538
% -------------------------------------------------------------------------
% Prueba de obstáculo hexagonal en Crazyflie con Fusión de Sensores:
% Flow Deck + Sistema de Captura de Movimiento
% =========================================================================

%% Añadir al path las carpetas de comandos usando una ruta relativa
addpath('../../Crazyflie-Matlab');
addpath('../../Robotat');

%% Conexión
robotat = robotat_connect();

%% Lectura de markers en obstáculos y generación de trayectoria
agent_id = 14;  
obstacle1_id = 11; 
goal_id = 12;

origin = robotat_get_pose(robotat, agent_id,"eulxyz");
obstacle1_point = robotat_get_pose(robotat, obstacle1_id,"eulxyz");
final_point = robotat_get_pose(robotat, goal_id,"eulxyz");
final_point(3) = final_point(3) + 0.2;

% Offset para marker 11 en obstáculo
obstacle1_point(6) = mod(obstacle1_point(6), 360);
obstacle1_point(6) = obstacle1_point(6) - 201;
if obstacle1_point(6) < 0
    obstacle1_point(6) = obstacle1_point(6) + 360;
end

save data_obstacle_1.mat origin final_point obstacle1_point
obstacle_point = obstacle1_point;

% Generación de trayectoria interpolada
% Geometría del obstáculo
diameter = 0.23;
radius = diameter/2;
obstacle_point(3) = obstacle_point(3) - radius;
theta = linspace(0, 2*pi, 7); 
hexagon_y = radius * cos(theta); 
hexagon_z = radius * sin(theta) + obstacle_point(3); 
apply_yaw = @(x, y, yaw) [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)] * [x; y];
yaw_obstacle = deg2rad(obstacle_point(6));  % Ángulo de yaw en radianes
rotated_hexagon = apply_yaw(zeros(size(hexagon_y)), hexagon_y, yaw_obstacle);

takeoff_point = origin(1:3) + [0,0,obstacle1_point(3) - radius];

obstacle_distance = 0.15;  
pre_obstacle = obstacle_point(1:3) - obstacle_distance * [cos(yaw_obstacle), sin(yaw_obstacle), 0];
post_obstacle = obstacle_point(1:3) + obstacle_distance * [cos(yaw_obstacle), sin(yaw_obstacle), 0];

% Crear la figura
figure;
hold on;
[x_floor, y_floor] = meshgrid(-1:0.1:1, -1:0.1:1);
z_floor = zeros(size(x_floor));
surf(x_floor, y_floor, z_floor, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5); % Dibujar el piso

% Graficar el obstáculo rotado con el yaw aplicado
plot3(obstacle_point(1) + rotated_hexagon(1, :), obstacle_point(2) + rotated_hexagon(2, :), hexagon_z, 'r', 'LineWidth', 2);
% Graficar el origen como un punto
plot3(origin(1), origin(2), origin(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(origin(1), origin(2), origin(3), 'Origin', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% Graficar el punto de takeoff como un punto
plot3(takeoff_point(1), takeoff_point(2), takeoff_point(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
text(takeoff_point(1), takeoff_point(2), takeoff_point(3), 'Takeoff', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% Graficar el punto final como un punto
plot3(final_point(1), final_point(2), final_point(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(final_point(1), final_point(2), final_point(3), 'Final Point', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% Graficar el punto pre-obstacle y post-obstacle
plot3(pre_obstacle(1), pre_obstacle(2), pre_obstacle(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(post_obstacle(1), post_obstacle(2), post_obstacle(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Trayectoria e interpolación
trajectory_points = [takeoff_point(1:3); pre_obstacle(1:3); obstacle_point(1:3); post_obstacle(1:3); final_point(1:3)];
n_interp = 15; 
t = 1:size(trajectory_points, 1); 
t_interp = linspace(1, t(end), n_interp); 
x = interp1(t, trajectory_points(:, 1), t_interp, 'pchip');
y = interp1(t, trajectory_points(:, 2), t_interp, 'pchip');
z = interp1(t, trajectory_points(:, 3), t_interp, 'pchip');
plot3(x, y, z, 'm-', 'LineWidth', 2); % Línea suave
plot3(x, y, z, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Puntos interpolados

% Configurar los ejes
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Trayectoria que pasa por el centro del obstáculo');
grid on;
axis equal;
xlim([-2 0]); % Ajustar límites en X
ylim([-1 1]); % Ajustar límites en Y
zlim([0 0.75]); % Ajustar límites en Z para ver desde el suelo hacia arriba
view(3);
hold off;

% Generar el array de la trayectoria sin el primer punto
x_interp = x(2:end);
y_interp = y(2:end);
z_interp = z(2:end);
trajectory = [x_interp', y_interp', z_interp'];

%% Seguimiento de trayectoria
% Despegue
velocity = 1.0;
crazyflie_takeoff(crazyflie_1, 0.5, velocity);
try
    pose = robotat_get_pose(robotat, agent_id, "eulxyz");
    crazyflie_set_position(crazyflie_1, pose(1), pose(2), pose(3));
catch
    disp("Error al actualizar posición")
end
pause(0.5);

% Seguimiento de la trayectoria
crazyflie_move_to_position(crazyflie_1, x(1), y(1), z(1), velocity);
try
    pose = robotat_get_pose(robotat, agent_id, "eulxyz");
    crazyflie_set_position(crazyflie_1, pose(1), pose(2), pose(3));
catch
    disp("Error al actualizar posición")
end
pause(0.5);

for i = 1:N
    crazyflie_move_to_position(crazyflie_1, x(i), y(i), z(i), velocity);
    %pause(0.01);
    try
        pose = robotat_get_pose(robotat, agent_id, "eulxyz");
        crazyflie_set_position(crazyflie_1, pose(1), pose(2), pose(3));
    catch
        disp("Error al actualizar posición")
    end
end

crazyflie_move_to_position(crazyflie_1, center(1), center(2), center(3), velocity);
try
    pose = robotat_get_pose(robotat, agent_id, "eulxyz");
    crazyflie_set_position(crazyflie_1, pose(1), pose(2), pose(3));
catch
    disp("Error al actualizar posición")
end

% Aterrizaje
crazyflie_land(crazyflie_1);
% Desconexión
crazyflie_disconnect(crazyflie_1);


%% Ejecución de prueba de 9despegue y aterrizaje
dron_id = 8;    
crazyflie_1 = crazyflie_connect(dron_id);
pause(1);
crazyflie_set_position(crazyflie_1, -0.3, 0, 0);
pause(1);
crazyflie_takeoff(crazyflie_1);
pause(3);

for i = 1:size(trajectory,1)
    crazyflie_move_to_position(crazyflie_1, trajectory(i,1), trajectory(i,2), trajectory(i,3));
    pause(0.1);
end

crazyflie_land(crazyflie_1);
pause(2);
crazyflie_disconnect(crazyflie_1);