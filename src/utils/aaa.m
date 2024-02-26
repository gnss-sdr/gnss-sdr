% Especifica la dirección IP y el puerto del sensor
ip_sensor = '192.30.95.130'; % Reemplaza con la IP del sensor
puerto_sensor = 23; % Reemplaza con el puerto del sensor

% Configura la frecuencia de muestreo del sensor (en Hz)
frecuencia_muestreo = 850; % Hz

% Calcula el tiempo de espera entre muestras
tiempo_espera = 1 / frecuencia_muestreo;

% Número de muestras a leer
numero_muestras_a_leer = 1000; % por ejemplo, lee 1000 muestras

% Crea un objeto UDP
udp_objeto = udp(ip_sensor, puerto_sensor);

% Configura el objeto UDP
set(udp_objeto, 'Timeout', 30); % Establece el tiempo de espera

% Abre la conexión UDP
fopen(udp_objeto);

% Inicializa la variable para almacenar los datos
datos_recibidos = zeros(1, numero_muestras_a_leer); % suponiendo que recibes datos escalares

% Lee los datos del sensor
for i = 1:numero_muestras_a_leer
    % Lee una muestra de datos binarios del sensor
    datos_recibidos(i) = fread(udp_objeto, 1, 'uint16'); % ajusta 'uint16' según el formato de tus datos
end

% Cierra la conexión UDP
fclose(udp_objeto);

% Limpia y elimina el objeto UDP
delete(udp_objeto);

% Ahora los datos recibidos están almacenados en la variable "datos_recibidos"
% Puedes trabajar con ellos según sea necesario