#codigo main
#
#

#
#

# main_program.py
import cv2
import threading
import serial
import time
from camera_config import video_capture_with_filters

cam_obj = None  # variable compartida entre hilos

# === UART CONFIG ===
UART_PORT = '/dev/ttyACM0'
UART_BAUD = 9600

try:
    uart = serial.Serial(
        port=UART_PORT,
        baudrate=UART_BAUD,
        timeout=1
    )
    print("[UART] Puerto UART abierto correctamente")
except serial.SerialException as e:
    print("[UART] Error al abrir puerto UART:", e)
    uart = None                  
                                                                                                               
# === HILO DE CÁMARA ===
def main_camera():
    global cam_obj
    cam = cv2.VideoCapture(0)
    cam_obj = video_capture_with_filters(cam, "None")
    cam_obj.start_camera()

# === HILO UART ===
def uart_loop():
    global cam_obj
    if uart is None:
        return

    while True:
        try:
            if cam_obj is None or not hasattr(cam_obj, 'detector') or cam_obj.detector is None:
                print("[UART] Esperando a que cam_obj.detector esté disponible...")
                time.sleep(0.5)
                continue

            cx = cam_obj.detector.cx
            cy = cam_obj.detector.cy
            flago = cam_obj.detector.band

            if cx is not None and cy is not None:
                posx = (3-len(str(cx)))*"0"+str(cx)
                fl = (178*25)/7.5
                d = (7.5*fl)/(cy+1)
                if d>99:
                    d=99
                dist = (2-len(str(int(d))))*"0"+str(int(d))
                mensaje = str(dist) + posx + str(flago)
                try:
                    mensa = int(mensaje)
                except
                :
                    mensaje = "000000\n"
                uart.write(mensaje.encode('utf-8'))
                time.sleep(0.1)
                print(f"[UART] Enviado: {mensaje.strip()}")
            else:
                print("[UART] Coordenadas aún no disponibles")

            if uart.in_waiting > 0:
                recibido = uart.readline().decode('utf-8').strip()
                print(f"[UART] Recibido: {recibido}")

            time.sleep(0.9)

        except Exception as e:
            print("[UART] Error:", e)
            break

    uart.close()

# === MAIN ===
if __name__ == "__main__":
    hilo_camera = threading.Thread(target=main_camera, daemon=True)
    hilo_uart = threading.Thread(target=uart_loop, daemon=True)
                                           
    hilo_camera.start()
    hilo_uart.start()

    hilo_camera.join()
    hilo_uart.join()                                                                                                                       