import cv2
import numpy as np
import math
import serial
import time

# ===================== CONFIGURACION ARDUINO =====================
PUERTO_SERIAL = 'COM11'
BAUDRATE = 115200
TIMEOUT = 0.01

try:
    arduino = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)
    print("✓ Serial conectado en", PUERTO_SERIAL)
except:
    print("✗ ERROR: No se pudo conectar en", PUERTO_SERIAL)
    arduino = None

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
        output = np.clip(output, -90, 90)
        self.prev_error = error
        return output

pid_x = PID(0.8, 0.05, 0.15)
pid_y = PID(0.8, 0.05, 0.15)
pid_z = PID(0.3, 0.02, 0.08, setpoint=2000)   # para el área (altura)

# ===================== PARAMETROS HOUGH (CALIBRADOS) =====================
dp        = 0.8
minDist   = 280
param1    = 165
param2    = 15
minRadius = 130
maxRadius = 211

# ===================== FILTRO EXPONENCIAL DEL PLATO =====================
alpha_plato = 0.8
plato_cx_f = plato_cy_f = plato_r_f = None

# ===================== FUNCION CUADRANTES RESPECTO AL PLATO =====================
def cuadrante_en_plato(cx_bola, cy_bola, plato_cx, plato_cy, plato_r):
    if plato_cx is None or plato_r is None:
        return "SIN_PLATO", 0.0, 0.0, (128,128,128)

    dx = cx_bola - plato_cx
    dy = cy_bola - plato_cy
    dist = math.hypot(dx, dy)

    # fuera del plato (error X,Y = 0 para no descontrolar)
    if dist > plato_r * 0.98:
        return "FUERA_PLATO", 0.0, 0.0, (0,0,255)

    # centro superior / inferior (20% del radio)
    radio_centro = plato_r * 0.2
    if dist < radio_centro:
        if dy < 0:
            return "CENTRO_SUP", 0.0, 0.0, (0,255,255)
        else:
            return "CENTRO_INF", 0.0, 0.0, (255,255,0)

    # ángulo 0° = derecha, antihorario
    ang = math.degrees(math.atan2(dy, dx))
    if ang < 0:
        ang += 360
    sector = int(ang // 60)

    if   sector == 0: cuadrante, color = "Cuadrante 1", (0,255,0)
    elif sector == 1: cuadrante, color = "Cuadrante 2", (0,165,255)
    elif sector == 2: cuadrante, color = "Cuadrante 3", (255,0,255)
    elif sector == 3: cuadrante, color = "Cuadrante 4", (255,255,0)
    elif sector == 4: cuadrante, color = "Cuadrante 5", (255,0,0)
    else:             cuadrante, color = "Cuadrante 6", (0,0,255)

    # ======= ERROR NORMALIZADO DEL CENTRO DEL PLATO A SU BORDE =======
    # magnitud 0 en centro, 1 en borde del plato
    mag = np.clip(dist / plato_r, 0.0, 1.0)
    errX = (dx / plato_r)       # normalizado por radio en X
    errY = (dy / plato_r)       # normalizado por radio en Y
    # si quieres limitar a [-1,1]:
    errX = np.clip(errX, -1.0, 1.0)
    errY = np.clip(errY, -1.0, 1.0)

    return cuadrante, errX, errY, color

# ===================== VIDEO =====================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

alpha_err = 0.6
errX_f = errY_f = 0.0

print("Presiona 'q' para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)

    # ---------- 1) DETECCION DEL PLATO ----------
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)

    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT,
        dp=dp, minDist=minDist,
        param1=param1, param2=param2,
        minRadius=minRadius, maxRadius=maxRadius
    )

    plato_cx = plato_cy = plato_r = None

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        radios = [r for (_,_,r) in circles]
        idx = np.argmax(radios)
        plato_cx, plato_cy, plato_r = circles[idx]

        if plato_cx_f is None:
            plato_cx_f, plato_cy_f, plato_r_f = plato_cx, plato_cy, plato_r
        else:
            plato_cx_f = int(alpha_plato*plato_cx_f + (1-alpha_plato)*plato_cx)
            plato_cy_f = int(alpha_plato*plato_cy_f + (1-alpha_plato)*plato_cy)
            plato_r_f  = int(alpha_plato*plato_r_f  + (1-alpha_plato)*plato_r)

        cv2.circle(frame, (plato_cx_f, plato_cy_f), plato_r_f, (0,255,0), 3)
        cv2.circle(frame, (plato_cx_f, plato_cy_f), 6, (0,255,0), -1)
        cv2.putText(frame, f"PLATO r={plato_r_f}px",
                    (plato_cx_f-80, plato_cy_f-plato_r_f-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    else:
        plato_cx_f = plato_cy_f = plato_r_f = None

    # ---------- 2) DETECCION PELOTA ROJA ----------
    red_low1  = np.array([0, 150, 50], np.uint8)
    red_high1 = np.array([8, 255, 255], np.uint8)
    red_low2  = np.array([170, 150, 50], np.uint8)
    red_high2 = np.array([179, 255, 255], np.uint8)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
        if 100 < area < 8000:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                if mejor_c is None or area > mejor_area:
                    mejor_c, mejor_cx, mejor_cy, mejor_area = c, cx, cy, area

    cuadrante = "SIN_BOLA"
    color_cuad = (0,0,255)
    errX = errY = 0.0

    # ---------- 3) ERRORES RESPECTO AL PLATO ----------
    if mejor_c is not None and plato_cx_f is not None:
        cuadrante, ex, ey, color_cuad = cuadrante_en_plato(
            mejor_cx, mejor_cy, plato_cx_f, plato_cy_f, plato_r_f
        )
        errX = alpha_err*errX_f + (1-alpha_err)*ex
        errY = alpha_err*errY_f + (1-alpha_err)*ey
        errX_f, errY_f = errX, errY

        cv2.circle(frame, (mejor_cx, mejor_cy), 8, color_cuad, -1)
        hull = cv2.convexHull(mejor_c)
        cv2.drawContours(frame, [hull], 0, color_cuad, 2)

    # ---------- 4) PID Y SERIAL ----------
    outX = pid_x.update(errX*100)   # error ya es [-1,1], lo escalas
    outY = pid_y.update(errY*100)
    outZ = pid_z.update(mejor_area)

    if arduino is not None:
        cmd = f"X:{int(outX)},Y:{int(outY)},Z:{int(outZ)}\n"
        arduino.write(cmd.encode())

    # ---------- 5) HUD ----------
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, f'CUADRANTE: {cuadrante}', (10, 30), font, 0.7, color_cuad, 2)
    cv2.putText(frame, f'errX:{errX:+.2f}  errY:{errY:+.2f}', (10, 55), font, 0.6, (0,255,0), 2)
    cv2.putText(frame, f'AREA:{int(mejor_area)}', (10, 80), font, 0.6, (255,255,0), 2)
    cv2.putText(frame, f'PID X:{int(outX)} Y:{int(outY)} Z:{int(outZ)}',
                (10, frame.shape[0]-20), font, 0.6, (0,255,0), 2)

    cv2.imshow('PLATO + CUADRANTES + PELOTA', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ===================== LIMPIEZA =====================
if arduino is not None:
    arduino.write(b"X:0,Y:0,Z:0\n")
    time.sleep(0.5)
    arduino.close()
cap.release()
cv2.destroyAllWindows()
print("✓ Sistema cerrado")
