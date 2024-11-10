% =========================================================================
% PROYECTO DE GRADUACIÓN: HERRAMIENTAS DE SOFTWARE PARA CRAZYFLIE
% Pablo Javier Caal Leiva - 20538
% -------------------------------------------------------------------------
% Prueba de obstáculo hexagonal en Crazyflie con Flow Deck
% =========================================================================

%% Añadir al path las carpetas de comandos usando una ruta relativa
addpath('../../Crazyflie-Matlab');
addpath('../../Robotat');

%% Generación de trayectoria interpolada
% Puntos importantes
origin = [-0.3, 0, 0];
takeoff_point = origin(1:3) + [0,0,0.3];
obstacle_point = [0, 0, 0.3, 0, 0, deg2rad(0)]; 
final_point = [0.3, 0, 0.3];

obstacle1_id = 11; 

% Geometría del obstáculo
diameter = 0.27;
radius = diameter / 2;
obstacle_point(3) = obstacle_point(3) - radius;
theta = linspace(0, 2*pi, 7); 
hexagon_y = radius * cos(theta); 
hexagon_z = radius * sin(theta) + obstacle_point(3); 
apply_yaw = @(x, y, yaw) [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)] * [x; y];
yaw_obstacle = obstacle_point(6);  % Ángulo de yaw en radianes
rotated_hexagon = apply_yaw(zeros(size(hexagon_y)), hexagon_y, yaw_obstacle);

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
trajectory_points = [takeoff_point; pre_obstacle; obstacle_point(1:3); post_obstacle; final_point];
n_interp = 20; 
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
title('Trayectoria que pasa por el centro del obstáculo');
grid on;
axis equal;
xlim([-0.75 0.75]); % Ajustar límites en X
ylim([-0.75 0.75]); % Ajustar límites en Y
zlim([0 0.5]); % Ajustar límites en Z para ver desde el suelo hacia arriba
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