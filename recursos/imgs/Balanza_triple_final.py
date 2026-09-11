import cv2
import numpy as np
import math
import serial
import time


# ===================== CONFIGURACION SERIAL =====================
PUERTO_SERIAL = 'COM11' # <-- AJUSTA ESTO AL PUERTO DE TU ESP32
BAUDRATE = 115200
TIMEOUT = 0.01


try:
    arduino = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)
    print("✓ Serial conectado en", PUERTO_SERIAL)
except Exception as e:
    print(f"✗ ERROR: No se pudo conectar en {PUERTO_SERIAL}. {e}")
    arduino = None
    
# ===================== PARAMETROS HOUGH (PLATO) =====================
dp       = 0.8
minDist  = 280
param1   = 165
param2   = 15
minRadius = 130
maxRadius = 211


# ===================== FILTRO EXPONENCIAL DEL PLATO =====================
alpha_plato = 0.9 
plato_cx_f = plato_cy_f = plato_r_f = None 


# ===================== COEFICIENTES DE SUAVIZADO DE ERROR =====================
alpha_err = 0.6
errX_f = errY_f = 0.0 


# ===================== VIDEO =====================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


# ===================== ANGULOS DE SERVO (INICIALES) =====================
BASE_ANGLE = 50.0 
MIN_ANGLE = 30.0  
MAX_ANGLE = 70.0   


COS_30 = math.cos(math.radians(30)) 
SIN_30 = math.sin(math.radians(30)) 


# ===================== PID BASICO =====================
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, dt=0.03):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.prev_error = 0
        self.integral = 0


    def update(self, measurement):
        error = self.setpoint - measurement
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -50, 50) 
        derivative = (error - self.prev_error) / self.dt
        
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        output = np.clip(output, -20, 20)
        self.prev_error = error
        return output


# Inicializar PIDs de inclinación
pid_dt = 0.03 
pid_x = PID(Kp=10.0, Ki=0.2, Kd=1.0, dt=pid_dt) 
pid_y = PID(Kp=10.0, Ki=0.2, Kd=1.0, dt=pid_dt)


# ===================== CINEMÁTICA DE 3 SERVOS =====================
def calcular_angulos_servos(theta_x, theta_y, base_angle, min_angle, max_angle):
    angle_1 = base_angle + theta_y 
    angle_2 = base_angle + (theta_x * COS_30) - (theta_y * SIN_30)
    angle_3 = base_angle - (theta_x * COS_30) - (theta_y * SIN_30)

    angle_1 = np.clip(angle_1, min_angle, max_angle)
    angle_2 = np.clip(angle_2, min_angle, max_angle)
    angle_3 = np.clip(angle_3, min_angle, max_angle)
    
    return int(angle_1), int(angle_2), int(angle_3)


# ===================== ENVIO DE CONFIGURACION AL ESP32 =====================
def enviar_config_angulos(base, min_ang, max_ang):
    """Envía la configuración de ángulos al ESP32"""
    if arduino is not None:
        try:
            comando_config = f"CFG:BASE:{base},MIN:{min_ang},MAX:{max_ang}\n"
            arduino.write(comando_config.encode('ascii'))
            print(f"✓ Config enviada: BASE={base}°, MIN={min_ang}°, MAX={max_ang}°")
        except Exception as e:
            print(f"Error al enviar config: {e}")


# ===================== CONFIGURACIÓN DE TRACKBARS =====================
def on_trackbar(val):
    pass


# Función para crear imagen de información con valores reales
def crear_imagen_info_pid(kp, ki, kd):
    img = np.zeros((150, 400, 3), dtype=np.uint8)
    img[:] = (40, 40, 40)  # Fondo gris oscuro
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "VALORES PID ACTUALES", (70, 30), font, 0.7, (255, 255, 255), 2)
    cv2.line(img, (20, 40), (380, 40), (100, 100, 100), 1)
    
    cv2.putText(img, f"Kp = {kp:.2f}", (50, 75), font, 0.8, (0, 255, 255), 2)
    cv2.putText(img, f"Ki = {ki:.3f}", (50, 105), font, 0.8, (0, 255, 255), 2)
    cv2.putText(img, f"Kd = {kd:.2f}", (50, 135), font, 0.8, (0, 255, 255), 2)
    
    return img


def crear_imagen_info_angulos(base, min_ang, max_ang):
    img = np.zeros((150, 400, 3), dtype=np.uint8)
    img[:] = (40, 40, 40)  # Fondo gris oscuro
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "ANGULOS DE SERVOS", (85, 30), font, 0.7, (255, 255, 255), 2)
    cv2.line(img, (20, 40), (380, 40), (100, 100, 100), 1)
    
    cv2.putText(img, f"Base = {base}.0 grados", (50, 75), font, 0.7, (255, 165, 0), 2)
    cv2.putText(img, f"Min  = {min_ang}.0 grados", (50, 105), font, 0.7, (255, 165, 0), 2)
    cv2.putText(img, f"Max  = {max_ang}.0 grados", (50, 135), font, 0.7, (255, 165, 0), 2)
    
    return img


# Crear ventana de control PID
cv2.namedWindow("PID Tuner")
cv2.resizeWindow("PID Tuner", 400, 150)

# Trackbars PID (sin texto de unidades, se muestra en la imagen)
cv2.createTrackbar("Kp x10", "PID Tuner", int(pid_x.Kp * 10), 500, on_trackbar)
cv2.createTrackbar("Ki x100", "PID Tuner", int(pid_x.Ki * 100), 100, on_trackbar)
cv2.createTrackbar("Kd x10", "PID Tuner", int(pid_x.Kd * 10), 50, on_trackbar)

# Crear ventana de control de Ángulos
cv2.namedWindow("Servo Angles")
cv2.resizeWindow("Servo Angles", 400, 150)

# Trackbars de ángulos (0-180 grados)
cv2.createTrackbar("Base", "Servo Angles", int(BASE_ANGLE), 180, on_trackbar)
cv2.createTrackbar("Min", "Servo Angles", int(MIN_ANGLE), 180, on_trackbar)
cv2.createTrackbar("Max", "Servo Angles", int(MAX_ANGLE), 180, on_trackbar)

# Enviar configuración inicial
enviar_config_angulos(int(BASE_ANGLE), int(MIN_ANGLE), int(MAX_ANGLE))

print("🎯 Detección de Plato y Errores de Pelota (Fondo Negro)")
print("📐 Ajusta los ángulos en tiempo real con los sliders")
print("Presiona 'q' para salir")

# Variables para detectar cambios en ángulos
prev_base = int(BASE_ANGLE)
prev_min = int(MIN_ANGLE)
prev_max = int(MAX_ANGLE)

# Bucle principal de control
while True:
    # --- LECTURA Y ACTUALIZACIÓN DEL PID ---
    kp_val = cv2.getTrackbarPos("Kp x10", "PID Tuner") / 10.0
    ki_val = cv2.getTrackbarPos("Ki x100", "PID Tuner") / 100.0
    kd_val = cv2.getTrackbarPos("Kd x10", "PID Tuner") / 10.0
    
    pid_x.Kp, pid_x.Ki, pid_x.Kd = kp_val, ki_val, kd_val
    pid_y.Kp, pid_y.Ki, pid_y.Kd = kp_val, ki_val, kd_val

    # --- LECTURA DE ÁNGULOS DESDE TRACKBARS ---
    current_base = cv2.getTrackbarPos("Base", "Servo Angles")
    current_min = cv2.getTrackbarPos("Min", "Servo Angles")
    current_max = cv2.getTrackbarPos("Max", "Servo Angles")
    
    # Mostrar valores reales en las ventanas de trackbars
    img_pid_info = crear_imagen_info_pid(kp_val, ki_val, kd_val)
    cv2.imshow("PID Tuner", img_pid_info)
    
    img_angle_info = crear_imagen_info_angulos(current_base, current_min, current_max)
    cv2.imshow("Servo Angles", img_angle_info)
    
    # Validar que Min < Base < Max
    if current_min >= current_max:
        current_max = current_min + 1
        cv2.setTrackbarPos("Max", "Servo Angles", current_max)
    
    if current_base < current_min:
        current_base = current_min
        cv2.setTrackbarPos("Base", "Servo Angles", current_base)
    elif current_base > current_max:
        current_base = current_max
        cv2.setTrackbarPos("Base", "Servo Angles", current_base)
    
    # Detectar cambios y enviar nueva configuración
    if (current_base != prev_base or current_min != prev_min or current_max != prev_max):
        enviar_config_angulos(current_base, current_min, current_max)
        prev_base, prev_min, prev_max = current_base, current_min, current_max
    
    # --- RESTO DE LÓGICA DE VISIÓN Y CONTROL ---
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    frame_display = frame.copy() 

    # --- 1) DETECCION DEL PLATO (HOUGH) ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT,
        dp=dp, minDist=minDist,
        param1=param1, param2=param2,
        minRadius=minRadius, maxRadius=maxRadius
    )

    plato_cx, plato_cy, plato_r = None, None, None

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        radios = [r for (_,_,r) in circles]
        idx = np.argmax(radios)
        plato_cx, plato_cy, plato_r = circles[idx]

        if plato_cx_f is None:
            plato_cx_f, plato_cy_f, plato_r_f = plato_cx, plato_cy, plato_r
        else:
            plato_cx_f = int(alpha_plato*plato_cx + (1-alpha_plato)*plato_cx_f)
            plato_cy_f = int(alpha_plato*plato_cy + (1-alpha_plato)*plato_cy_f)
            plato_r_f  = int(alpha_plato*plato_r + (1-alpha_plato)*plato_r_f)

        cv2.circle(frame_display, (plato_cx_f, plato_cy_f), plato_r_f, (0,255,0), 3)
        cv2.circle(frame_display, (plato_cx_f, plato_cy_f), 6, (0,255,0), -1) # Centro
        
        # --- NUEVO CÍRCULO: ZONA MUERTA ---
        DEADZONE_RADIUS = 10 # Radio en píxeles para la zona muerta (ajusta si es necesario)
        cv2.circle(frame_display, (plato_cx_f, plato_cy_f), DEADZONE_RADIUS, (0,165,255), 2)
        
        cv2.putText(frame_display, f"PLATO DETECTADO (r={plato_r_f})", 
                             (plato_cx_f - 120, plato_cy_f - plato_r_f - 10), 
                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    else:
        plato_cx_f = plato_cy_f = plato_r_f = None
        cv2.putText(frame_display, "PLATO NO DETECTADO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    # --- A) ENMASCARAMIENTO CIRCULAR (FONDO NEGRO) ---
    if plato_cx_f is not None:
        mask_plato = np.zeros(frame.shape[:2], dtype="uint8")
        cv2.circle(mask_plato, (plato_cx_f, plato_cy_f), plato_r_f - 5, 255, -1) 
        frame_deteccion = cv2.bitwise_and(frame, frame, mask=mask_plato)
    else:
        frame_deteccion = frame

    # --- 2) DETECCION PELOTA ROJA (COLOR / CENTROIDE) ---
    red_low1  = np.array([0, 150, 50], np.uint8)
    red_high1 = np.array([8, 255, 255], np.uint8)
    red_low2  = np.array([170, 150, 50], np.uint8)
    red_high2 = np.array([179, 255, 255], np.uint8)

    hsv = cv2.cvtColor(frame_deteccion, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, red_low1, red_high1)
    mask2 = cv2.inRange(hsv, red_low2, red_high2)
    mask = cv2.add(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mejor_c = None
    mejor_cx = mejor_cy = 0
    mejor_area = 0

    for c in contornos:
        area = cv2.contourArea(c)
        if 100 < area < 5000:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                
                if mejor_c is None or area > mejor_area:
                    mejor_c, mejor_cx, mejor_cy, mejor_area = c, cx, cy, area

    # --- 3) CALCULO Y SUAVIZADO DE ERROR ---
    errX_raw = 0.0
    errY_raw = 0.0
    pelota_detectada = False
    
    if mejor_c is not None and plato_cx_f is not None:
        dist_al_centro = math.hypot(mejor_cx - plato_cx_f, mejor_cy - plato_cy_f)
        
        if dist_al_centro < plato_r_f:
            pelota_detectada = True
            
            errX_raw = mejor_cx - plato_cx_f 
            errY_raw = mejor_cy - plato_cy_f 

            if plato_r_f > 0:
                 errX_norm = errX_raw / plato_r_f
                 errY_norm = errY_raw / plato_r_f
            else:
                 errX_norm, errY_norm = 0.0, 0.0

            errX_f = alpha_err * errX_norm + (1-alpha_err) * errX_f
            errY_f = alpha_err * errY_norm + (1-alpha_err) * errY_f

            cv2.circle(frame_display, (mejor_cx, mejor_cy), 8, (0,0,255), -1)
            cv2.line(frame_display, (mejor_cx, mejor_cy), (plato_cx_f, plato_cy_f), (255,0,0), 2)
            
        else:
            errX_f, errY_f = 0.0, 0.0
            # RESETEAR PID cuando no hay pelota
            pid_x.integral = 0.0
            pid_y.integral = 0.0
            pid_x.prev_error = 0.0
            pid_y.prev_error = 0.0
    else:
        errX_f, errY_f = 0.0, 0.0
        # RESETEAR PID cuando no hay pelota
        pid_x.integral = 0.0
        pid_y.integral = 0.0
        pid_x.prev_error = 0.0
        pid_y.prev_error = 0.0

    # --- 4) CONTROL PID Y ENVÍO SERIAL (🎯 LÓGICA CORREGIDA) ---
    errX_input = errX_f * 100
    errY_input = errY_f * 100
    
    # Zona muerta para estabilidad
    DEADZONE_ERR_INPUT = 5.0
    if abs(errX_input) < DEADZONE_ERR_INPUT:
        errX_input = 0.0
    if abs(errY_input) < DEADZONE_ERR_INPUT:
        errY_input = 0.0

    # 🎯 CORRECCIÓN CLAVE: INVERTIR SEÑAL DE CONTROL
    theta_X = -pid_x.update(errX_input)  # ← NEGATIVO: COMPENSA el error
    theta_Y = -pid_y.update(errY_input)  # ← NEGATIVO: COMPENSA el error
    
    # Limitar inclinaciones extremas
    theta_X = np.clip(theta_X, -25, 25)
    theta_Y = np.clip(theta_Y, -25, 25)

    A1, A2, A3 = calcular_angulos_servos(theta_X, theta_Y, current_base, current_min, current_max)

    comando = f"A1:{A1},A2:{A2},A3:{A3}\n"
    
    if arduino is not None:
        try:
            arduino.write(comando.encode('ascii'))
        except Exception as e:
            print(f"Error al enviar datos: {e}")
            
    # --- 5) HUD FINAL ---
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    cv2.putText(frame_display, f"Kp:{kp_val:.1f} Ki:{ki_val:.2f} Kd:{kd_val:.1f}", 
                (10, frame_display.shape[0]-120), font, 0.5, (255,255,0), 2)
    
    cv2.putText(frame_display, f"Base:{current_base} Min:{current_min} Max:{current_max}", 
                (10, frame_display.shape[0]-95), font, 0.5, (255,165,0), 2)
    
    cv2.putText(frame_display, f"Angulos: A1:{A1} A2:{A2} A3:{A3}", 
                (10, frame_display.shape[0]-70), font, 0.5, (255,255,0), 2)
    
    if pelota_detectada:
        cv2.putText(frame_display, 
                    f'Inclinacion X,Y: ({theta_X:+.1f} deg, {theta_Y:+.1f} deg)', 
                    (10, frame_display.shape[0]-45), font, 0.5, (0,255,0), 2)
        cv2.putText(frame_display, 
                    f'Error Norm X,Y: ({errX_f:+.2f}, {errY_f:+.2f})', 
                    (10, frame_display.shape[0]-20), font, 0.5, (0,255,255), 2)
    else:
        cv2.putText(frame_display, "PELOTA NO DETECTADA O FUERA DEL PLATO", 
                    (10, frame_display.shape[0]-20), font, 0.5, (0,165,255), 2)

    cv2.imshow('CONTROL DE BALANZA (PID ACTIVO)', frame_display)
    cv2.imshow('Imagen de Deteccion (Solo Plato Visible)', frame_deteccion)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
    time.sleep(pid_dt) 

# ===================== LIMPIEZA =====================
print("\n✓ Sistema cerrado. Centrando Servos...")
if arduino is not None:
    comando_cierre = f"A1:{current_base},A2:{current_base},A3:{current_base}\n"
    arduino.write(comando_cierre.encode('ascii'))
    time.sleep(0.5)
    arduino.close()
    
cap.release()
cv2.destroyAllWindows()
