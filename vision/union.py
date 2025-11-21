import cv2
from ultralytics import YOLO
import time
import serial
import numpy as np

# ============== CONFIG YOLO ==============
MODEL_PATH = "runs/detect/train6/weights/best.pt"

CAMERA_SOURCE = 2
CONF_THRESHOLD = 0.35

CLASS_COLORS = {
    0: (0, 255, 0),
    2: (0, 255, 255),
    1: (0, 0, 255),
    3: (255, 255, 255)
}

# --- COORDENADAS DEL RECORTE (ROI) ---
x1, y1 = 400, 350
x2, y2 = 1000, 800

# ============== CONFIG UART ==============
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# ============== SECUENCIA DE PLANTAS ==============
NUM_PLANTAS = 6

# Tiempos de viaje desde el punto anterior HASTA cada planta
TRAVEL_TIMES = [2.3, 0.72, 0.85, 0.92, 0.9, 3]
OBSERVATION_TIME = 3  # tiempo viendo cada planta antes de detectar


def recortar_roi(frame):
    """Devuelve solo el recorte ROI desde el frame general."""
    h, w = frame.shape[:2]
    x1_clip = max(0, min(x1, w - 1))
    x2_clip = max(0, min(x2, w))
    y1_clip = max(0, min(y1, h - 1))
    y2_clip = max(0, min(y2, h))
    roi = frame[y1_clip:y2_clip, x1_clip:x2_clip]
    return roi


def detectar_en_recorte(model, frame):
    """Recorta el ROI, corre YOLO y devuelve (class_name, confidence, roi)."""
    roi = recortar_roi(frame)

    results = model(roi, conf=CONF_THRESHOLD, verbose=False)
    r = results[0]
    boxes = r.boxes

    if boxes is None or len(boxes) == 0:
        return None, None, roi

    # Dibujar todo y elegir mejor detección
    best_conf = -1.0
    best_name = None
    for box in boxes:
        bx1, by1, bx2, by2 = box.xyxy[0].cpu().numpy().astype(int)
        conf = float(box.conf[0])
        cls = int(box.cls[0])

        class_name = r.names[cls] if hasattr(r, "names") and cls in r.names else f"class {cls}"
        color = CLASS_COLORS.get(cls, (255, 0, 0))

        cv2.rectangle(roi, (bx1, by1), (bx2, by2), color, 2)
        label = f"{class_name} {conf:.2f}"
        cv2.putText(roi, label, (bx1, max(by1 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if conf > best_conf:
            best_conf = conf
            best_name = class_name

    return best_name, best_conf, roi


def construir_ui(frame, roi, status_lines, plant_states):
    """
    Construye una sola imagen con:
    - Izquierda: frame grande
    - Derecha arriba: ROI
    - Derecha abajo: panel de texto con estado general + estado de plantas
    """
    # Tamaño de la ventana final (full HD)
    CANVAS_W = 1920
    CANVAS_H = 1080

    # Crear lienzo negro
    canvas = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)

    # ---- Sección izquierda: frame principal grande ----
    # Redimensionamos el frame a 1440x1080 (ocupa todo el alto)
    main_w, main_h = 1440, 1080
    frame_resized = cv2.resize(frame, (main_w, main_h))
    canvas[0:main_h, 0:main_w] = frame_resized

    # ---- Sección derecha arriba: ROI ----
    roi_w, roi_h = 480, 360
    if roi is not None and roi.size != 0:
        roi_resized = cv2.resize(roi, (roi_w, roi_h))
    else:
        roi_resized = np.zeros((roi_h, roi_w, 3), dtype=np.uint8)
        cv2.putText(roi_resized, "ROI sin imagen",
                    (40, roi_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    canvas[0:roi_h, main_w:main_w + roi_w] = roi_resized

    # ---- Sección derecha abajo: panel de texto ----
    panel_h = CANVAS_H - roi_h  # 1080 - 360 = 720
    panel = np.zeros((panel_h, roi_w, 3), dtype=np.uint8)

    # Líneas que se van a mostrar:
    # - Estado general (modo, planta actual, FPS)
    # - Separador
    # - Estado de plantas (P1, P2, ...)

    lines = []
    lines.extend(status_lines)
    lines.append("-" * 20)
    lines.extend(plant_states)

    y0 = 40
    dy = 30
    for i, line in enumerate(lines):
        y = y0 + i * dy
        if y >= panel_h - 10:
            break
        cv2.putText(panel, line, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    canvas[roi_h:CANVAS_H, main_w:main_w + roi_w] = panel

    return canvas


def main():
    print("📦 Cargando modelo YOLOv11...")
    model = YOLO(MODEL_PATH)

    # ---- Cámara ----
    cap = cv2.VideoCapture(CAMERA_SOURCE)
    if not cap.isOpened():
        print(f"❌ No se pudo abrir la cámara: {CAMERA_SOURCE}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1580)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1020)

    window_name = "Monitor Hidrovida - YOLOv11"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    # Pantalla completa (ideal para HDMI)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    prev_time = 0

    # ---- UART ----
    print(f"🔌 Abriendo puerto serie: {SERIAL_PORT} @ {BAUDRATE}")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)

    print("✅ Cámara iniciada.")
    print("➡ Presiona 'q' para salir.")
    print("➡ Presiona el botón ANALISIS en Node-RED para disparar la secuencia.\n")

    # ====== ESTADO DE LA SECUENCIA ======
    secuencia_activa = False
    planta_actual = 0          # 1..6 cuando está activa
    resultados = []
    proximo_tiempo = 0.0       # timestamp para la próxima detección

    # ROI e info para el panel derecho
    last_roi = None
    status_lines = []

    # Solo estados de planta para la pantalla negra
    plant_states = []  # Ej: ["P1: SANA (0.92)", "P2: ENFERMA (0.81)", ...]

    while True:
        # Leer frame
        ret, frame = cap.read()
        if not ret:
            print("❌ No se pudo leer frame")
            break

        current_time = time.time()
        fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
        prev_time = current_time

        # ================== MODO NORMAL (YOLO EN VIVO) ==================
        if not secuencia_activa:
            results_yolo = model(frame, conf=CONF_THRESHOLD, verbose=False)
            r = results_yolo[0]
            boxes = r.boxes

            if boxes is not None:
                for box in boxes:
                    x1b, y1b, x2b, y2b = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])

                    class_name = r.names[cls] if hasattr(r, "names") and cls in r.names else f"class {cls}"
                    color = CLASS_COLORS.get(cls, (255, 255, 255))

                    cv2.rectangle(frame, (x1b, y1b), (x2b, y2b), color, 2)
                    label = f"{class_name} {conf:.2f}"
                    cv2.putText(frame, label, (x1b, max(y1b - 5, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # ROI de referencia en modo normal
            last_roi = recortar_roi(frame)

            status_lines = [
                "Modo: MONITOREO",
                "Esperando comando 'ANALISIS'...",
                f"FPS: {fps:.1f}"
            ]

            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.putText(frame, "Esperando comando UART: 'ANALISIS'", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # ================== MODO SECUENCIA ==================
        else:
            cv2.putText(frame, f"Secuencia - Planta {planta_actual}/{NUM_PLANTAS}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            status_lines = [
                "Modo: SECUENCIA",
                f"Planta actual: {planta_actual}/{NUM_PLANTAS}",
                f"FPS: {fps:.1f}",
                "Analizando ROI en cada parada..."
            ]

            # ¿Es momento de detectar esta planta?
            if current_time >= proximo_tiempo and planta_actual <= NUM_PLANTAS:
                print(f"\n📸 Detectando planta {planta_actual}...")

                class_name, conf, roi = detectar_en_recorte(model, frame)
                last_roi = roi  # actualizar ROI que se mostrará en la UI

                if class_name is None:
                    estado = f"P{planta_actual}: SIN DETECCION"
                    print(f"🌱 {estado}")
                    resultados.append((f"Planta {planta_actual}", "SIN DETECCIÓN", 0.0))
                    msg = f"P{planta_actual}:SIN_DETECCION"
                else:
                    estado = f"P{planta_actual}: {class_name} ({conf:.2f})"
                    print(f"🌱 {estado}")
                    resultados.append((f"Planta {planta_actual}", class_name, conf))
                    msg = f"P{planta_actual}:{class_name}:{conf:.2f}"

                # Guardar SOLO estado de planta para la pantalla negra
                plant_states.append(estado)

                # 🔹 Enviar resultado por UART a la ESP32
                try:
                    ser.write((msg + "\n").encode())
                    print(f"📤 Enviado a ESP32 por UART: {msg}")
                except Exception as e:
                    print(f"⚠️ Error enviando UART: {e}")

                # Configurar siguiente planta o terminar
                if planta_actual < NUM_PLANTAS:
                    planta_actual += 1
                    idx = planta_actual - 1  # índice 0..5 para TRAVEL_TIMES
                    total_espera = TRAVEL_TIMES[idx] + OBSERVATION_TIME
                    proximo_tiempo = current_time + total_espera
                    print(f"⏳ Viaje a planta {planta_actual}: {TRAVEL_TIMES[idx]} s + {OBSERVATION_TIME} s observación")
                else:
                    # Fin secuencia
                    print("\n✅ SECUENCIA COMPLETADA. RESUMEN:")
                    print("==========================================")
                    for planta, clase, c in resultados:
                        if clase in ["SIN DETECCIÓN", "SIN FRAME"]:
                            print(f"{planta}: {clase}")
                        else:
                            print(f"{planta}: {clase} (conf={c:.2f})")
                    print("==========================================\n")

                    # Mensaje final por UART
                    try:
                        ser.write(b"FIN_SECUENCIA\n")
                        print("📤 Enviado a ESP32: FIN_SECUENCIA")
                    except Exception as e:
                        print(f"⚠️ Error enviando FIN_SECUENCIA: {e}")

                    secuencia_activa = False
                    planta_actual = 0

        # ================== CONSTRUIR UI ÚNICA ==================
        ui_image = construir_ui(frame, last_roi, status_lines, plant_states)
        cv2.imshow(window_name, ui_image)

        # ================== LECTURA UART ==================
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            line = ""

        if line:
            print(f"📥 UART recibido: {line}")
            if "ANALISIS" in line.upper() and not secuencia_activa:
                secuencia_activa = True
                resultados = []
                plant_states = []  # limpiar estados de plantas al iniciar nueva secuencia
                planta_actual = 1
                total_espera = TRAVEL_TIMES[0] + OBSERVATION_TIME
                proximo_tiempo = current_time + total_espera
                print(f"🚀 Secuencia iniciada. P1 en {TRAVEL_TIMES[0]} s + {OBSERVATION_TIME} s observación")

        # ================== TECLA SALIR ==================
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("👋 Cámara y UART cerrados.")


if __name__ == "__main__":
    main()
