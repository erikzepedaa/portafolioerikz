import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import math

# Configuración serial ARDUINO UNO
PUERTO_SERIAL = 'COM11'
BAUDRATE = 11520
TIMEOUT = 0.01

# Slicers interactivos para área
min_area = 100
max_area = 8000
alto_slicer = 30

# Variables globales para mouse
mouse_x, mouse_y = 0, 0

# Función para encontrar puerto Arduino automáticamente
def encontrar_arduino():
    puertos = serial.tools.list_ports.comports()
    for puerto in puertos:
        if 'Arduino' in puerto.description or 'CH340' in puerto.description or 'USB-SERIAL' in puerto.description:
            return puerto.device
    return None

# Función slicer interactivo
def dibujar_slicer(frame, y_pos, valor, label, min_val, max_val, color=(0,255,0)):
    ancho = frame.shape[1] - 20
    x_inicio = 10
    
    cv2.rectangle(frame, (x_inicio, y_pos), (x_inicio + ancho, y_pos + alto_slicer), (50,50,50), -1)
    
    proporcion = (valor - min_val) / (max_val - min_val)
    ancho_activo = int(proporcion * ancho)
    cv2.rectangle(frame, (x_inicio, y_pos), (x_inicio + ancho_activo, y_pos + alto_slicer), color, -1)
    
    cv2.putText(frame, f'{label}: {int(valor)}', (x_inicio, y_pos-5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    
    cv2.putText(frame, f'[{min_val}-{max_val}]', (x_inicio + ancho + 10, y_pos + 22), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 1)

# 🔥 FUNCIÓN 6 CUADRANTES POLARES (60° cada uno)
def obtener_cuadrante_6(cx, cy, centrox, centroy):
    """
    Divide en 6 cuadrantes polares cada 60° + CENTRO_SUP/INF
    Ángulos: 0°(Der), 60°, 120°, 180°(Izq), 240°, 300°
    """
    dx = cx - centrox
    dy = cy - centroy
    
    # Distancia desde centro (para detectar centro)
    distancia = math.sqrt(dx**2 + dy**2)
    radio_centro = 50  # Píxeles para considerar "centro"
    
    if distancia < radio_centro:
        if dy < 0:  # CENTRO_SUPERIOR
            return "CENTRO_SUP", 0.0, 0.0, (0, 255, 255)
        else:       # CENTRO_INFERIOR
            return "CENTRO_INF", 0.0, 0.0, (255, 255, 0)
    
    # Ángulo polar (0°=derecha, sentido antihorario)
    angulo = math.atan2(dy, dx) * 180 / math.pi
    if angulo < 0:
        angulo += 360
    
    # Dividir en 6 sectores de 60°
    sector = int(angulo // 60)
    
    if sector == 0:
        cuadrante = "Cuadrante 6"
        color_cuad = (0, 0, 255)  # Rojo
    elif sector == 1:
        cuadrante = "Cuadrante 5"
        color_cuad = (0, 165, 255)  # Naranja
    elif sector == 2:
        cuadrante = "Cuadrante 4"
        color_cuad = (255, 0, 255)  # Magenta
    elif sector == 3:
        cuadrante = "Cuadrante 3"
        color_cuad = (255, 255, 0)  # Cian
    elif sector == 4:
        cuadrante = "Cuadrante 2"
        color_cuad = (255, 0, 0)  # Rosa
    else:  # sector == 5
        cuadrante = "Cuadrante 1"
        color_cuad = (0, 255, 0)  # Verde
    
    # Errores normalizados por distancia máxima
    dist_max = 250  # Radio máximo para normalización
    error_magnitud = np.clip(distancia / dist_max, 0, 1.0)
    
    # Dirección del error (vector unitario)
    errorX_norm = (dx / distancia) * error_magnitud if distancia > 0 else 0
    errorY_norm = (dy / distancia) * error_magnitud if distancia > 0 else 0
    
    return cuadrante, errorX_norm, errorY_norm, color_cuad

# Mouse callback para ventana SLICER
def mouse_callback_slicer(event, x, y, flags, param):
    global min_area, max_area, mouse_x, mouse_y
    mouse_x, mouse_y = x, y
    
    if event == cv2.EVENT_MOUSEWHEEL:
        if 100 <= y <= 160:
            delta = 50 if event > 0 else -50
            if 100 <= y <= 130:
                min_area = np.clip(min_area + delta, 50, 5000)
            else:
                max_area = np.clip(max_area + delta * 2, 500, 15000)

# Inicializar comunicación serial
print("🔍 Buscando Arduino en COM11...")
try:
    arduino = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)
    print("✓ Serial conectado en COM11")
except:
    print("✗ ERROR: No se pudo conectar en COM11")
    exit()

# Clase PID
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, dt=0.03):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.prev_error = 0
        self.integral = 0
        self.output = 0
    
    def update(self, measurement):
        error = self.setpoint - measurement
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -50, 50)
        derivative = (error - self.prev_error) / self.dt
        self.output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.output = np.clip(self.output, -90, 90)
        self.prev_error = error
        return self.output

# Inicializar PIDs
pid_x = PID(Kp=0.8, Ki=0.05, Kd=0.15)
pid_y = PID(Kp=0.8, Ki=0.05, Kd=0.15)
pid_area = PID(Kp=0.3, Ki=0.02, Kd=0.08, setpoint=2000)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
alpha = 0.6
prev_errorX, prev_errorY = 0, 0

print("\n🎯 BALANZA PID 3D - 6 CUADRANTES POLARES")
print("VENTANA 1: 'Camera' → 6 sectores 60° + CENTRO_SUP/INF")
print("VENTANA 2: 'SLICER' → ↑↓Rueda | A/Z:Min | S/X:Max")

# 2 VENTANAS
ventana_camera = 'Camera - 6 CUADRANTES POLARES'
ventana_slicer = 'SLICER CONTROL'

cv2.namedWindow(ventana_camera)
cv2.namedWindow(ventana_slicer)
cv2.setMouseCallback(ventana_slicer, mouse_callback_slicer)

frame_slicer = np.zeros((200, 320, 3), dtype=np.uint8)
frame_slicer.fill(20)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    centrox = frame.shape[1] // 2
    centroy = frame.shape[0] // 2

    # Detección roja
    redbajo1 = np.array([0, 150, 50], np.uint8)
    redalto1 = np.array([8, 255, 255], np.uint8)
    redbajo2 = np.array([170, 150, 50], np.uint8)
    redalto2 = np.array([179, 255, 255], np.uint8)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, redbajo1, redalto1)
    mask2 = cv2.inRange(hsv, redbajo2, redalto2)
    mask = cv2.add(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mejor_contorno = None
    mejor_cx, mejor_cy, mejor_area = None, None, 0
    cuadrante_actual = "CENTRO"
    errorX_norm, errorY_norm = 0, 0

    for c in contornos:
        area = cv2.contourArea(c)
        if min_area < area < max_area:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if mejor_contorno is None or area > mejor_area:
                    mejor_contorno = c
                    mejor_cx, mejor_cy = cx, cy
                    mejor_area = area

    if mejor_contorno is not None:
        cuadrante_actual, errorX_norm, errorY_norm, color_cuad = obtener_cuadrante_6(
            mejor_cx, mejor_cy, centrox, centroy
        )
        
        errorX = alpha * errorX_norm + (1-alpha) * prev_errorX
        errorY = alpha * errorY_norm + (1-alpha) * prev_errorY
        prev_errorX, prev_errorY = errorX, errorY
    else:
        errorX, errorY = 0, 0
        arduino.write(b"X:0,Y:0,Z:0\n")

    outputX = pid_x.update(errorX * 100)
    outputY = pid_y.update(errorY * 100)
    outputArea = pid_area.update(mejor_area)

    comando = f"X:{int(outputX)},Y:{int(outputY)},Z:{int(outputArea)}\n"
    arduino.write(comando.encode())

    # 🔥 VENTANA 1: CÁMARA + 6 LÍNEAS POLARES
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # Círculo centro + área sensible
    cv2.circle(frame, (centrox, centroy), 10, (0, 255, 0), -1)
    cv2.circle(frame, (centrox, centroy), 50, (0, 255, 0), 1)  # Radio CENTRO
    
    # 🔥 6 LÍNEAS RADIALES CADA 60° (150px largo)
    radio_lineas = 150
    for angulo in range(0, 360, 60):
        rad = math.radians(angulo)
        x_fin = int(centrox + radio_lineas * math.cos(rad))
        y_fin = int(centroy + radio_lineas * math.sin(rad))
        cv2.line(frame, (centrox, centroy), (x_fin, y_fin), (0, 255, 0), 2)

    if mejor_contorno is not None:
        cv2.circle(frame, (int(mejor_cx), int(mejor_cy)), 8, color_cuad, -1)
        hull = cv2.convexHull(mejor_contorno)
        cv2.drawContours(frame, [hull], 0, color_cuad, 3)

        cv2.putText(frame, f'{cuadrante_actual}', (10, 30), font, 0.7, color_cuad, 2)
        cv2.putText(frame, f'X:{errorX_norm:+.2f}', (10, 55), font, 0.6, (0,255,0), 2)
        cv2.putText(frame, f'Y:{errorY_norm:+.2f}', (10, 80), font, 0.6, (0,255,0), 2)
        cv2.putText(frame, f'A:{int(mejor_area)}', (10, 105), font, 0.6, (255,255,0), 2)
        cv2.putText(frame, f'PID: X{int(outputX)} Y{int(outputY)} Z{int(outputArea)}', 
                   (10, frame.shape[0]-30), font, 0.5, (0,255,0), 2)
    else:
        cv2.putText(frame, 'NO BALL', (10, 105), font, 0.6, (0,0,255), 2)

    # 🔥 VENTANA 2: SLICERS
    frame_slicer.fill(20)
    dibujar_slicer(frame_slicer, 100, min_area, 'MIN_AREA', 50, 5000, (0,255,0))
    dibujar_slicer(frame_slicer, 135, max_area, 'MAX_AREA', 500, 15000, (0,150,255))
    
    cv2.circle(frame_slicer, (mouse_x, mouse_y), 5, (255,255,255), -1)
    
    font_slicer = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame_slicer, f'↑↓Rueda sobre slicers', (10, 50), font_slicer, 0.4, (255,255,0), 1)
    cv2.putText(frame_slicer, f'A/Z:Min±100 S/X:Max±200', (10, 75), font_slicer, 0.4, (255,255,0), 1)
    cv2.putText(frame_slicer, f'Rango: {min_area}-{max_area}', (10, 180), font_slicer, 0.5, (0,255,255), 2)

    cv2.imshow(ventana_camera, frame)
    cv2.imshow(ventana_slicer, frame_slicer)

    tecla = cv2.waitKey(1) & 0xFF
    if tecla == ord('a'): min_area = max(50, min_area - 100); print(f"MIN_AREA: {min_area}")
    if tecla == ord('z'): min_area = min(5000, min_area + 100); print(f"MIN_AREA: {min_area}")
    if tecla == ord('s'): max_area = max(500, max_area - 200); print(f"MAX_AREA: {max_area}")
    if tecla == ord('x'): max_area = min(15000, max_area + 200); print(f"MAX_AREA: {max_area}")
    if tecla == ord('q'):
        break

# Limpieza
arduino.write(b"X:0,Y:0,Z:0\n")
time.sleep(0.5)
cap.release()
arduino.close()
cv2.destroyAllWindows()
print("✓ Sistema cerrado correctamente")
