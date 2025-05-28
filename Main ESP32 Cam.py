import network
import time
import camera
import urequests

# === CONFIGURACIÓN ===
SSID = 'S10+'        # ✅ Asegúrate de que el nombre es correcto y visible
PASSWORD = '123456789'            # ✅ Asegúrate de que la contraseña esté bien escrita
SERVER_URL = 'http://192.168.182.48:5000/upload'  # Asegúrate de que Flask esté activo

# === CONEXIÓN A Wi-Fi CON TIMEOUT ===
MAX_ATTEMPTS = 10
attempt = 0

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

while not wlan.isconnected() and attempt < MAX_ATTEMPTS:
    print(f"📡 Conectando a Wi-Fi... Intento {attempt + 1}")
    time.sleep(1)
    attempt += 1

if not wlan.isconnected():
    raise OSError("❌ No se pudo conectar a la red Wi-Fi")
else:
    print("✅ Conectado a Wi-Fi:", wlan.ifconfig())

# === INICIALIZAR CÁMARA ===
try:
    time.sleep(2)  # Espera para evitar errores por falta de RAM
    camera.init(0, format=camera.JPEG)
    print("📸 Cámara inicializada correctamente")
except Exception as e:
    print("⚠️ Error al inicializar cámara:", e)

# === FUNCIÓN PARA ENVIAR IMAGEN Y RECIBIR RESPUESTA ===
def send_image_and_get_response(img):
    try:
        headers = {'Content-Type': 'application/octet-stream'}
        response = urequests.post(SERVER_URL, data=img, headers=headers)
        result = response.text.strip()
        response.close()
        print("📤 Imagen enviada. Respuesta del servidor:", result)
        return result
    except Exception as e:
        print("⚠️ Error al enviar imagen:", e)
        return None

# === BUCLE PRINCIPAL ===
while True:
    try:
        print("📷 Capturando imagen...")
        img = camera.capture()
        if img:
            diagnosis = send_image_and_get_response(img)
            if diagnosis:
                if "enfermo" in diagnosis.lower():
                    print("⚠️ Persona posiblemente enferma. Activar protocolo.")
                    # Aquí puedes encender un LED, enviar alerta, etc.
                elif "sano" in diagnosis.lower():
                    print("✅ Persona sana.")
        else:
            print("⚠️ No se pudo capturar imagen")

        time.sleep(0.5)

    except Exception as e:
        print("⚠️ Error en bucle principal:", e)
