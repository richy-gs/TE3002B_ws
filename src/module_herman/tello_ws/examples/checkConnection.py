from djitellopy import Tello

def check_drone_connection():
    try:
        # Crear la instancia del Tello
        tello = Tello()
        
        # Intentar conectar con el drone
        tello.connect()
        
        # Intentar obtener el nivel de batería para confirmar la conexión
        battery = tello.get_battery()
        if battery is not None:
            print("Conexión exitosa con el drone. Nivel de batería: {}%".format(battery))
        else:
            print("Conexión establecida, pero no se pudo obtener el nivel de batería.")
            
        # Finalizar la conexión
        tello.end()
    except Exception as e:
        print("No se pudo conectar con el drone:", e)

if __name__ == '__main__':
    check_drone_connection()