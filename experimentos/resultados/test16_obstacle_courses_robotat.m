%% =========================================================================
% PROYECTO DE GRADUACIÓN: HERRAMIENTAS DE SOFTWARE PARA CRAZYFLIE
% Pablo Javier Caal Leiva - 20538
% -------------------------------------------------------------------------
% Prueba de curso de obstáculos hexagonales en Crazyflie con Flow Deck y 
% lecturas de Robotat para corregir errores en posición
% =========================================================================

%% Añadir al path las carpetas de comandos usando una ruta relativa
addpath('../../Crazyflie-Matlab');
addpath('../../Robotat');

%% Lectura de markers en obstáculos y generación de trayectoria
robotat = robotat_connect();

%%
agent_id = 12;  
obstacle1_id = 11; 
obstacle2_id = 15; 
obstacle3_id = 13; 
goal_id = 14;

origin = robotat_get_pose(robotat, agent_id,"eulxyz");
obstacle1_point = robotat_get_pose(robotat, obstacle1_id,"eulxyz");
obstacle2_point = robotat_get_pose(robotat, obstacle2_id,"eulxyz");
obstacle3_point = robotat_get_pose(robotat, obstacle3_id,"eulxyz");
final_point = robotat_get_pose(robotat, goal_id,"eulxyz");

% Offset para marer 11 en obstáculo
obstacle1_point(6) = mod(obstacle1_point(6), 360);
obstacle1_point(6) = obstacle1_point(6) - 201;
if obstacle1_point(6) < 0
    obstacle1_point(6) = obstacle1_point(6) + 360;
end

% Offset para marer 15 en obstáculo
obstacle2_point(6) = mod(obstacle2_point(6), 360);
obstacle2_point(6) = obstacle2_point(6) +315;
if obstacle2_point(6) >= 360
    obstacle2_point(6) = obstacle2_point(6) - 360;
end

% Offset para marker 13 en osbtáculo
obstacle3_point(6) = mod(obstacle3_point(6), 360);
obstacle3_point(6) = obstacle3_point(6) - 318;
if obstacle3_point(6) < 0
    obstacle3_point(6) = obstacle3_point(6) + 360;
end

save data_obstacle_course1.mat origin final_point obstacle1_point obstacle2_point obstacle3_point


%% Generación de trayectoria interpolada
% Puntos importantes
% origin = [-0.3, 0, 0];
takeoff_point = origin(1:3) + [0,0,0.3];
% obstacle1_point = [0.25, 0.5, 0.3, 0, 0, deg2rad(-45)]; 
% obstacle2_point = [0.5, -0.5, 0.3, 0, 0, deg2rad(0)]; 
% obstacle3_point = [0.75, 0, 0.3, 0, 0, deg2rad(90)]; 
% final_point = [1, 0, 0.3];

% Geometría del obstáculo
diameter = 0.27;
radius = diameter / 2;
obstacle1_point(3) = obstacle1_point(3) - radius;
obstacle2_point(3) = obstacle2_point(3) - radius;
obstacle3_point(3) = obstacle3_point(3) - radius;

theta = linspace(0, 2*pi, 7); 
hexagon_y = radius * cos(theta); 
hexagon1_z = radius * sin(theta) + obstacle1_point(3);
hexagon2_z = radius * sin(theta) + obstacle2_point(3);
hexagon3_z = radius * sin(theta) + obstacle3_point(3);

apply_yaw = @(x, y, yaw) [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)] * [x; y];
yaw_obstacle1 = deg2rad(obstacle1_point(6));  
yaw_obstacle2 = deg2rad(obstacle2_point(6));  
yaw_obstacle3 = deg2rad(obstacle3_point(6));  % Ángulo de yaw en radianes

rotated_hexagon1 = apply_yaw(zeros(size(hexagon_y)), hexagon_y, yaw_obstacle1);
rotated_hexagon2 = apply_yaw(zeros(size(hexagon_y)), hexagon_y, yaw_obstacle2);
rotated_hexagon3 = apply_yaw(zeros(size(hexagon_y)), hexagon_y, yaw_obstacle3);

obstacle_distance = 0.15;  
pre_obstacle1 = obstacle1_point(1:3) - obstacle_distance * [cos(yaw_obstacle1), sin(yaw_obstacle1), 0];
post_obstacle1 = obstacle1_point(1:3) + obstacle_distance * [cos(yaw_obstacle1), sin(yaw_obstacle1), 0];
pre_obstacle2 = obstacle2_point(1:3) - obstacle_distance * [cos(yaw_obstacle2), sin(yaw_obstacle2), 0];
post_obstacle2 = obstacle2_point(1:3) + obstacle_distance * [cos(yaw_obstacle2), sin(yaw_obstacle2), 0];
pre_obstacle3 = obstacle3_point(1:3) - obstacle_distance * [cos(yaw_obstacle3), sin(yaw_obstacle3), 0];
post_obstacle3 = obstacle3_point(1:3) + obstacle_distance * [cos(yaw_obstacle3), sin(yaw_obstacle3), 0];

% Crear la figura
figure;
hold on;
[x_floor, y_floor] = meshgrid(-1:0.1:1, -1:0.1:1);
z_floor = zeros(size(x_floor));
surf(x_floor, y_floor, z_floor, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5); % Dibujar el piso

% Graficar los obstáculos rotados con el yaw aplicado
plot3(obstacle1_point(1) + rotated_hexagon1(1, :), obstacle1_point(2) + rotated_hexagon1(2, :), hexagon1_z, 'r', 'LineWidth', 2);
plot3(obstacle2_point(1) + rotated_hexagon2(1, :), obstacle2_point(2) + rotated_hexagon2(2, :), hexagon2_z, 'r', 'LineWidth', 2);
plot3(obstacle3_point(1) + rotated_hexagon3(1, :), obstacle3_point(2) + rotated_hexagon3(2, :), hexagon3_z, 'r', 'LineWidth', 2);

% Graficar el origen como un punto
plot3(origin(1), origin(2), origin(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(origin(1), origin(2), origin(3), 'Origin', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% Graficar el punto de takeoff como un punto
plot3(takeoff_point(1), takeoff_point(2), takeoff_point(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
text(takeoff_point(1), takeoff_point(2), takeoff_point(3), 'Takeoff', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% Graficar el punto final como un punto
plot3(final_point(1), final_point(2), final_point(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(final_point(1), final_point(2), final_point(3), 'Final Point', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Graficar los puntos pre-obstacle y post-obstacle
plot3(pre_obstacle1(1), pre_obstacle1(2), pre_obstacle1(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(post_obstacle1(1), post_obstacle1(2), post_obstacle1(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(pre_obstacle2(1), pre_obstacle2(2), pre_obstacle2(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(post_obstacle2(1), post_obstacle2(2), post_obstacle2(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(pre_obstacle3(1), pre_obstacle3(2), pre_obstacle3(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(post_obstacle3(1), post_obstacle3(2), post_obstacle3(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Trayectoria e interpolación
trajectory_points = [takeoff_point; pre_obstacle1; obstacle1_point(1:3); post_obstacle1; ...
                     pre_obstacle2; obstacle2_point(1:3); post_obstacle2; ...
                     pre_obstacle3; obstacle3_point(1:3); post_obstacle3; ...
                     final_point(1:3)];
n_interp = 15; 
t = 1:size(trajectory_points, 1); 
t_interp = linspace(1, t(end), n_interp); 
x_interp = interp1(t, trajectory_points(:, 1), t_interp, 'pchip');
y_interp = interp1(t, trajectory_points(:, 2), t_interp, 'pchip');
z_interp = interp1(t, trajectory_points(:, 3), t_interp, 'pchip');
plot3(x_interp, y_interp, z_interp, 'm-', 'LineWidth', 2); % Línea suave
plot3(x_interp, y_interp, z_interp, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Puntos interpolados

% Configurar los ejes
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Trayectoria que pasa por el centro de los obstáculos');
grid on;
axis equal;
xlim([-2 2]); % Ajustar límites en X
ylim([-2.5 0]); % Ajustar límites en Y
zlim([0 2]); % Ajustar límites en Z para ver desde el suelo hacia arriba
view(3);
hold off;

% Generar el array de la trayectoria sin el primer punto
x_interp = x_interp(2:end);
y_interp = y_interp(2:end);
z_interp = z_interp(2:end);
trajectory = [x_interp', y_interp', z_interp'];


%% Ejecución de prueba de 9despegue y aterrizaje
dron_id = 8;    
crazyflie_1 = crazyflie_connect(dron_id);
pause(1);

robotat_update_crazyflie_position(crazyflie_1, robotat, agent_id);
pause(1);

crazyflie_takeoff(crazyflie_1);
pause(2);

for i = 1:size(trajectory,1)
    crazyflie_move_to_position(crazyflie_1, trajectory(i,1), trajectory(i,2), trajectory(i,3));
    robotat_update_crazyflie_position(crazyflie_1, robotat, agent_id);
    pause(0.25);
end

crazyflie_land(crazyflie_1);
pause(2);
crazyflie_disconnect(crazyflie_1);