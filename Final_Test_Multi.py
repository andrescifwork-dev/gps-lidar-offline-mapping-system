#FUNCIONA AL 100, DISTANCIAS NO REALES PARA PRUEBA.
from multiprocessing import Process, Queue
from math import cos, sin, radians
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import geopandas as gpd
from shapely.geometry import Point
import serial
import pynmea2
import winsound
import time

# MAPA
# Ruta al archivo GeoJSON
ruta_geojson = 'C:/Users/DELL OPTIPLEX/Documents/ANDRES FLACO/LiDAR PY/MapaCompletoUart_T1/ALAMOS 1/map/roads.geojson'

# Crear un GeoDataFrame para el marcador
marcador = gpd.GeoDataFrame(geometry=[Point(0, 0)])  # Inicializar en el origen
marcador.crs = "EPSG:4326"  # Establecer el sistema de coordenadas del marcador

# Crear un GeoDataFrame para el marcador del obstáculo
marcador_obstaculo = gpd.GeoDataFrame(geometry=[Point(0, 0)])  # Inicializar en el origen
marcador_obstaculo.crs = "EPSG:4326"  # Establecer el sistema de coordenadas del marcador del obstáculo

# Cargar el GeoJSON con geopandas
datos_mapa = gpd.read_file(ruta_geojson)

# Crear una figura y ejes
fig, ax = plt.subplots(figsize=(10, 6))  # Modifica el tamaño según tus preferencias
ax.set_title('Mapa con GPS y Obstáculo en Tiempo Real')

# Añadir el mapa
datos_mapa.plot(ax=ax)

# Añadir el marcador inicial con una flecha
marcador_plot = ax.scatter(0, 0, marker='o', color='green', s=100, label='Unidad')  # Utilizar una flecha como marcador

# Añadir el marcador del obstáculo
marcador_obstaculo_plot = ax.scatter(0, 0, marker='o', color='red', s=100, label='Obstáculo')

# Añadir leyenda
plt.legend()

alarma_activa = False


# MAPA
def update_map(latitud, longitud, vel, rumbo, distancia_laser):
    try:
        # Agrega "-" a longitud si no lo tiene.
        if longitud > 0:
            longitud = -longitud  # Agregar el signo negativo

        # Actualizar la posición del marcador
        marcador.geometry = [Point(longitud, latitud)]

        # Actualizar la posición del marcador en el gráfico
        marcador_plot.set_offsets([(longitud, latitud)])

        # Si rumbo es None, asignarle el valor 0
        if rumbo is None:
            rumbo = 0
        else:
            # Si rumbo es un valor diferente de None y diferente de 0, guardar ese valor
            if rumbo != 0:
                update_map.last_rumbo = rumbo

        # Rotar el marcador según el rumbo
        transform = Affine2D().rotate_deg(rumbo)
        marcador_plot.set_transform(transform)

        # Para rastrear el estado de la alarma
        global alarma_activa

        # Actualizar la posición del marcador del obstáculo (si hay obstáculo)
        #if distancia_laser is not None and 20 < distancia_laser <= 350:
        if distancia_laser > 20 and distancia_laser <= 350:
            # Calcular la posición del marcador del obstáculo en función de la dirección del auto y la distancia
            latitud_obstaculo = latitud + distancia_laser * 0.00001
            longitud_obstaculo = longitud

            # Calcular la rotación del vector entre el vehículo y el obstáculo
            rotacion_rad = radians(rumbo)
            cos_yaw = cos(rotacion_rad)
            sin_yaw = sin(rotacion_rad)

            # Calcular la nueva posición del marcador del obstáculo
            longitud_relativa = longitud_obstaculo - longitud
            latitud_relativa = latitud_obstaculo - latitud
            longitud_obstaculo_nuevo = longitud + (cos_yaw * longitud_relativa - sin_yaw * latitud_relativa)
            latitud_obstaculo_nuevo = latitud + (sin_yaw * longitud_relativa + cos_yaw * latitud_relativa)

            marcador_obstaculo.geometry = [Point(longitud_obstaculo_nuevo, latitud_obstaculo_nuevo)]
            marcador_obstaculo_plot.set_offsets([(longitud_obstaculo_nuevo, latitud_obstaculo_nuevo)])
            marcador_obstaculo_plot.set_visible(True)

            # Activar la alarma si no está activa
            if not alarma_activa:
                alarma_activa = True
                winsound.PlaySound('alarma.wav', winsound.SND_FILENAME | winsound.SND_ASYNC)

        else:
            # No hay obstáculo, ocultar el marcador del obstáculo
            marcador_obstaculo_plot.set_visible(False)
            # Desactivar la alarma si está activa
            if alarma_activa:
                alarma_activa = False
                winsound.PlaySound(None, winsound.SND_FILENAME)

            # Calcular los nuevos límites del mapa para centrarse en el marcador
            distancia_visual = 0.002
            limite_min_x = longitud - distancia_visual
            limite_max_x = longitud + distancia_visual
            limite_min_y = latitud - distancia_visual
            limite_max_y = latitud + distancia_visual

            # Actualizar los límites del eje x e y para centrarse en el marcador
            ax.set_xlim([limite_min_x, limite_max_x])
            ax.set_ylim([limite_min_y, limite_max_y])

            # Mostrar el mapa con los nuevos marcadores
            plt.draw()
    except TypeError:
        # Ignorar la excepción TypeError cuando los valores son None
        pass


def read_data(q):
    # Inicializar las variables con valores predeterminados
    latitud = 0
    longitud = 0
    vel = 0
    rumbo = 0
    distance_cm = 0

    # Inicializar variables con valores predeterminados
    latitud_obstaculo = 0
    longitud_obstaculo = 0
    longitud_obstaculo_nuevo = 0
    latitud_obstaculo_nuevo = 0

    try:
        ser = serial.Serial('COM8', 19200, timeout=0.3)
        puerto_gps = serial.Serial('COM10', 9600, timeout=0.9)
        while True:
            #Inicializar las variables con valores predeterminados
            """latitud = 0
            longitud = 0
            vel = 0
            rumbo = 0
            distance_cm = None"""

            # Leer datos del GPS
            linea = puerto_gps.readline().decode('utf-8').strip()
            if linea:
                data = linea.split(',')
                if data[0] == '$GPRMC' and len(data) >= 13 and data[2] == 'A':
                    try:
                        latitud = float(data[3][:2]) + float(data[3][2:]) / 60
                        longitud = float(data[5][:3]) + float(data[5][3:]) / 60
                        vel = float(data[7]) * 1.852 if data[7] else None
                        rumbo = float(data[8]) if data[8] else None
                    except ValueError:
                        pass
                elif data[0] == '$GPGGA' and len(data) >= 15 and data[6].isdigit() and int(data[6]) > 0:
                    try:
                        latitud = float(data[2][:2]) + float(data[2][2:]) / 60
                        longitud = float(data[4][:3]) + float(data[4][3:]) / 60
                    except ValueError:
                        pass

            # Leer datos del sensor de distancia
            ser.write(b'\xAA\x00\x00\x20\x00\x01\x00\x00\x21')
            ser.reset_input_buffer()
            data = ser.readline()
            if len(data) > 0:
                if hex(data[0]) == '0xee':
                    print(data)
                else:
                    distance_mm = int.from_bytes(data[6:10], byteorder='big', signed=False)
                    distance_cm = distance_mm / 10  # Convertir milímetros a centímetros

            # Colocar los datos en la cola
            q.put((latitud, longitud, vel, rumbo, distance_cm))

    except KeyboardInterrupt:
        print("Programa terminado por el usuario.")
        puerto_gps.close()
        ser.close()


if __name__ == "__main__":
    # Crear la cola
    q = Queue()

    # Crear el proceso
    p = Process(target=read_data, args=(q,))

    # Iniciar el proceso
    p.start()

    try:
        while True:
            # Obtener los resultados de la cola
            if not q.empty():
                latitud, longitud, vel, rumbo, distance_cm = q.get()


                # Llamar a la función para actualizar el mapa con los nuevos datos
                update_map(latitud, longitud, vel, rumbo, distance_cm)
                plt.pause(0.01)  # Permitir que matplotlib actualice el gráfico
    except KeyboardInterrupt:
        print("Programa terminado por el usuario.")

    # Asegurarse de que el proceso haya terminado
    p.join()