% =========================================================================
% EXPERIMENTOS CON CRAZYFLIE 2.1
% -------------------------------------------------------------------------
% Experimento 7: Vuelo a través de obstáculo con fusión de sensores
% =========================================================================




%% Parte 3: Ejecución de vuelo en Crazyflie con Flow Deck
% Recordatorio: Previo a ejecutar esta sección debe conectar el dispositivo 
% Crazyradio en algún puerto USB de su ordenador y encender el Crazyflie. 

% Importante, antes de ejecutar esta sección de código debe asegurarse que
% el dron tiene espacio físico suficiente para completar la trayectoria.

% Secuencia del experimento:
% Conexión con Crazyflie
dron_id = 8; 
crazyflie_1 = crazyflie_connect(dron_id); 
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