from flask import Flask, render_template, Response, request
import cv2
import numpy as np
import time
import threading
from deepface import DeepFace

# Inicializa la aplicación Flask
app = Flask(__name__)

# Carga el clasificador de rostros de OpenCV
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Variables globales para almacenar el último fotograma capturado y un candado para evitar condiciones de carrera
latest_frame = None
frame_lock = threading.Lock()

def diagnosticar_por_emocion(rostro):
    """
    Analiza la emoción dominante en un rostro usando DeepFace.
    Si la emoción pertenece a una lista de emociones negativas, devuelve 'Enfermo'; de lo contrario, 'Sano'.
    """
    try:
        resultado = DeepFace.analyze(rostro, actions=['emotion'], enforce_detection=False)
        emocion = resultado[0]['dominant_emotion']
        print(f"[DEBUG] Emoción detectada: {emocion}")

        if emocion in ['sad', 'angry', 'fear', 'disgust']:
            return "Enfermo"
        else:
            return "Sano"
    except Exception as e:
        print(f"[ERROR] No se pudo analizar la emoción: {e}")
        return "Desconocido"

@app.route('/')
def index():
    """
    Ruta principal que devuelve la página HTML.
    """
    return render_template('index.html')

@app.route('/upload', methods=['POST'])
def upload():
    """
    Recibe una imagen vía POST (desde la ESP32-CAM o cliente),
    la decodifica, rota, analiza y devuelve el estado emocional detectado.
    """
    global latest_frame
    image_bytes = request.data
    np_array = np.frombuffer(image_bytes, np.uint8)
    frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

    if frame is not None:
        # Rota la imagen 180 grados para compensar la orientación de la cámara
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Guarda el último fotograma de manera segura
        with frame_lock:
            latest_frame = frame.copy()

        # Convierte a escala de grises y detecta rostros
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))

        # Analiza cada rostro detectado y devuelve el estado
        for (x, y, w, h) in faces:
            rostro = frame[y:y+h, x:x+w]
            estado = diagnosticar_por_emocion(rostro)
            return estado  # Envía el estado al cliente

        return "Sin rostro detectado"
    else:
        return "Error al procesar imagen", 400

def generate_frames():
    """
    Genera fotogramas continuamente desde la última imagen recibida,
    analiza los rostros en tiempo real, dibuja rectángulos y estado emocional,
    y los transmite como una secuencia MJPEG.
    """
    global latest_frame
    while True:
        if latest_frame is not None:
            # Copia y redimensiona el último fotograma
            with frame_lock:
                small_frame = cv2.resize(latest_frame, (640, 480))

            # Convierte a escala de grises y detecta rostros
            gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))

            # Analiza cada rostro detectado y dibuja resultados
            for (x, y, w, h) in faces:
                rostro = small_frame[y:y+h, x:x+w]
                estado = diagnosticar_por_emocion(rostro)

                # Define color del rectángulo según el estado emocional
                color = (0, 255, 0) if estado == "Sano" else (0, 0, 255)
                cv2.rectangle(small_frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(small_frame, estado, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Codifica y transmite el fotograma como JPEG
            _, buffer = cv2.imencode('.jpg', small_frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # Espera si aún no hay imágenes recibidas
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    """
    Ruta que proporciona el stream de video en tiempo real (formato MJPEG).
    """
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Inicia el servidor Flask en todas las interfaces de red (IP local del PC) en el puerto 5000
    app.run(host='0.0.0.0', port=5000, debug=True)
