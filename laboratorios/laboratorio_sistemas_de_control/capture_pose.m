function capture_pose(~, ~, robotat, marker_id)
    global pos_data time_data timer_flag;

    % Obtener datos de posición del marcador Crazyflie usando Robotat
    pose = robotat_get_pose(robotat, marker_id, 'eulxyz');
    x = pose(1); % Posición en X
    y = pose(2); % Posición en Y
    z = pose(3); % Posición en Z
    
    % Capturar el tiempo actual
    current_time = toc; % Tiempo relativo al inicio del vuelo
    
    % Almacenar datos
    pos_data = [pos_data; x, y, z];
    time_data = [time_data; current_time];
    
    % Controlar cuando se desea detener el timer
    if ~timer_flag
        stop(timerfindall); % Detiene el timer
    end
end