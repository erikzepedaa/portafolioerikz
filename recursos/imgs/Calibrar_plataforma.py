import cv2
import numpy as np
import time

# ====== PARAMETROS INICIALES HOUGHCIRCLES (AJUSTADOS PARA TU PLATO ≈30cm a 40cm) ======
dp        = 1.0      # 1.0–1.2 recomendado
minDist   = 320      # ~ 1.5–2.0 * radio_px
param1    = 150      # Canny alto (bordes limpios)
param2    = 32       # Votos mínimos (más estricto)
minRadius = 150      # radio aprox - margen
maxRadius = 230      # radio aprox + margen

alto_slicer = 35
slicer_y = 80

# Variables globales para mouse
mouse_x, mouse_y = 0, 0

print("🎯 CALIBRADOR HoughCircles - PLATO CIRCULAR (con suavizado)")
print("CONTROLES:")
print("  ↑↓ Rueda mouse sobre slicers")
print("  Q/A  dp    | D/E  minDist")
print("  Z/X  param1| C/V  param2")
print("  B/N  minR  | ,/. maxR")
print("  ESPACIO: Imprimir valores óptimos | ESC: Salir")

def dibujar_slicer(frame, y_pos, valor, label, min_val, max_val, color=(0,255,0)):
    ancho = frame.shape[1] - 40
    x_inicio = 20

    cv2.rectangle(frame, (x_inicio, y_pos), (x_inicio + ancho, y_pos + alto_slicer), (60,60,60), -1)

    proporcion = (valor - min_val) / (max_val - min_val)
    proporcion = np.clip(proporcion, 0, 1)
    ancho_activo = int(proporcion * ancho)
    cv2.rectangle(frame, (x_inicio, y_pos+5), (x_inicio + ancho_activo, y_pos + alto_slicer-5), color, -1)

    cv2.putText(frame, f'{label}: {valor:.1f}', (x_inicio, y_pos-8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    cv2.putText(frame, f'[{min_val:.1f}-{max_val:.1f}]', (x_inicio + ancho + 10, y_pos + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,180,180), 1)

def mouse_callback(event, x, y, flags, param):
    global dp, minDist, param1, param2, minRadius, maxRadius, mouse_x, mouse_y
    mouse_x, mouse_y = x, y

    if event == cv2.EVENT_MOUSEWHEEL:
        delta = 0.05 if event > 0 else -0.05
        y_slicer = slicer_y

        if y_slicer <= y <= y_slicer + 6*45:
            if y_slicer <= y <= y_slicer + 35:           # DP
                dp = np.clip(dp + delta, 0.8, 2.0)
            elif y_slicer+45 <= y <= y_slicer+80:        # minDist
                minDist = np.clip(minDist + delta*50, 100, 500)
            elif y_slicer+90 <= y <= y_slicer+125:       # param1
                param1 = np.clip(param1 + delta*10, 50, 250)
            elif y_slicer+135 <= y <= y_slicer+170:      # param2
                param2 = np.clip(param2 + delta*5, 15, 80)
            elif y_slicer+180 <= y <= y_slicer+215:      # minRadius
                minRadius = np.clip(minRadius + delta*10, 80, 260)
            else:                                        # maxRadius
                maxRadius = np.clip(maxRadius + delta*10, 150, 350)

ventana_calibrador = 'HOUGH CIRCLES CALIBRADOR'
cv2.namedWindow(ventana_calibrador)
cv2.setMouseCallback(ventana_calibrador, mouse_callback)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# frame slicers mismo tamaño que la cámara para poder hacer hstack
frame_slicers = np.zeros((480, 640, 3), dtype=np.uint8)
frame_slicers.fill(30)

# ====== SUAVIZADO (FILTRO EXPONENCIAL DEL CIRCULO) ======
alpha_c = 0.8   # mientras más alto, más estable pero más lento al cambiar
plato_cx_f = plato_cy_f = plato_r_f = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT,
        dp=float(dp), minDist=int(minDist),
        param1=int(param1), param2=int(param2),
        minRadius=int(minRadius), maxRadius=int(maxRadius)
    )

    num_circles = 0
    mejor_plato = None

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        num_circles = len(circles)

        areas = [r*r for _, _, r in circles]
        mejor_idx = np.argmax(areas)
        mejor_plato = circles[mejor_idx]

        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 100, 0), 1)
            cv2.circle(frame, (x, y), 3, (0, 100, 0), -1)

        # suavizado sobre el mejor círculo
        x, y, r = mejor_plato
        if plato_cx_f is None:
            plato_cx_f, plato_cy_f, plato_r_f = x, y, r
        else:
            plato_cx_f = int(alpha_c * plato_cx_f + (1-alpha_c) * x)
            plato_cy_f = int(alpha_c * plato_cy_f + (1-alpha_c) * y)
            plato_r_f  = int(alpha_c * plato_r_f  + (1-alpha_c) * r)

        # dibujar círculo filtrado en verde grueso
        cv2.circle(frame, (plato_cx_f, plato_cy_f), plato_r_f, (0,255,0), 3)
        cv2.circle(frame, (plato_cx_f, plato_cy_f), 6, (0,255,0), -1)
        cv2.putText(frame, f"PLATO r={plato_r_f}px", (plato_cx_f-80, plato_cy_f-plato_r_f-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, f"CIRCULOS: {num_circles}", (10, 30), font, 0.7, (255,255,255), 2)
    if plato_cx_f is not None:
        cv2.putText(frame, f"CENTRO: ({plato_cx_f},{plato_cy_f})", (10, 55), font, 0.6, (0,255,0), 2)
    else:
        cv2.putText(frame, "AJUSTA PARAMETROS", (10, 55), font, 0.6, (0,0,255), 2)

    # ====== PANEL DE SLICERS ======
    frame_slicers.fill(30)
    y_pos = slicer_y
    dibujar_slicer(frame_slicers, y_pos, dp, 'DP', 0.8, 2.0, (0,255,255))
    y_pos += 45
    dibujar_slicer(frame_slicers, y_pos, minDist, 'minDist', 100, 500, (255,255,0))
    y_pos += 45
    dibujar_slicer(frame_slicers, y_pos, param1, 'param1', 50, 250, (0,255,0))
    y_pos += 45
    dibujar_slicer(frame_slicers, y_pos, param2, 'param2', 15, 80, (255,0,255))
    y_pos += 45
    dibujar_slicer(frame_slicers, y_pos, minRadius, 'minR', 80, 260, (255,165,0))
    y_pos += 45
    dibujar_slicer(frame_slicers, y_pos, maxRadius, 'maxR', 150, 350, (0,165,255))

    cv2.circle(frame_slicers, (mouse_x, mouse_y), 6, (255,255,255), -1)
    cv2.putText(frame_slicers, '↑↓ Rueda | Q/A/D/E/Z/X/C/V/B/N/,/.', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
    cv2.putText(frame_slicers,
                f'VALORES: dp={dp:.1f}  minDist={int(minDist)}  p1={int(param1)}  p2={int(param2)}',
                (20, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

    frame_combinado = np.hstack([frame, frame_slicers])
    cv2.imshow(ventana_calibrador, frame_combinado)

    tecla = cv2.waitKey(1) & 0xFF
    if tecla == ord('q'): dp = np.clip(dp - 0.1, 0.8, 2.0)
    if tecla == ord('a'): dp = np.clip(dp + 0.1, 0.8, 2.0)
    if tecla == ord('d'): minDist = np.clip(minDist - 20, 100, 500)
    if tecla == ord('e'): minDist = np.clip(minDist + 20, 100, 500)
    if tecla == ord('z'): param1 = np.clip(param1 - 10, 50, 250)
    if tecla == ord('x'): param1 = np.clip(param1 + 10, 50, 250)
    if tecla == ord('c'): param2 = np.clip(param2 - 5, 15, 80)
    if tecla == ord('v'): param2 = np.clip(param2 + 5, 15, 80)
    if tecla == ord('b'): minRadius = np.clip(minRadius - 10, 80, 260)
    if tecla == ord('n'): minRadius = np.clip(minRadius + 10, 80, 260)
    if tecla == ord(','): maxRadius = np.clip(maxRadius - 10, 150, 350)
    if tecla == ord('.'): maxRadius = np.clip(maxRadius + 10, 150, 350)

    if tecla == 32:  # ESPACIO
        print("\n=========== VALORES ACTUALES ===========")
        print(f"dp={dp}, minDist={int(minDist)}, param1={int(param1)}, param2={int(param2)}, "
              f"minRadius={int(minRadius)}, maxRadius={int(maxRadius)}")
        print("========================================")

    if tecla == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()

print("\nCopia esto a tu código principal de detección del plato:")
print(f"circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, "
      f"dp={dp}, minDist={int(minDist)}, param1={int(param1)}, param2={int(param2)}, "
      f"minRadius={int(minRadius)}, maxRadius={int(maxRadius)})")
