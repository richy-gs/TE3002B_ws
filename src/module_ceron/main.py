import sys

sys.path.insert(0, "/home/roli_005/module_ceron/coppeliaSim_ws/src/CoppeliaSim")

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Conectar al cliente
client = RemoteAPIClient()
sim = client.getObject("sim")

# Obtener el objeto del sensor de visión
sensor1Handle = sim.getObject("/visionSensor")

# Leer los datos del sensor
result, detectionState, auxPackets = sim.readVisionSensor(sensor1Handle)

# Imprimir los resultados
print("Resultado de lectura:", result)
print("Estado de detección:", detectionState)
print("Paquetes auxiliares:", auxPackets)
