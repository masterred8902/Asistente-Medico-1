from umqtt.simple import MQTTClient
from machine import Pin, PWM, I2C, ADC, reset, time_pulse_us
import network
import time
import json
import ubinascii
import urequests
import _thread
import struct
from ens160_aht21 import ENS160, AHT21
from max30102 import MAX30102
from ssd1306 import SSD1306_I2C

# Configuración inicial
SSID = 'S10+'
PASSWORD = '123456789'   
FIREBASE_URL = "https://sistemasprogramables-7eeea-default-rtdb.firebaseio.com"
FIREBASE_SECRET = "ddgR3eg6ERaVo0lnPKGz9MDCS4CO7jkoHdUCGbVf"
MQTT_BROKER = "test.mosquitto.org"
MQTT_PORT = 1883
CLIENT_ID = "esp32-" + ubinascii.hexlify(network.WLAN().config('mac')).decode()

# MQTT Topics
TOPIC_CONTROL_MOTORES = b"iot/control/motores"
TOPIC_CONTROL_AIRE = b"iot/control/aire"
TOPIC_CONTROL_ECG = b"iot/control/ecg"
TOPIC_CONTROL_MAX = b"iot/control/oximetroMaX30102"
TOPIC_CONTROL_MLX = b"iot/control/mlx90614"
TOPIC_CONTROL_ULTRASONICO = b"iot/control/ultrasonico"
TOPIC_CONTROL_SERVO = b"iot/control/servo"
TOPIC_CONTROL_SEGUIMIENTO = b"iot/control/seguimiento"  # Nuevo topic para seguimiento
TOPIC_SENSOR_DATA = b"iot/sensor/aire"
TOPIC_SENSOR_ECG = b"iot/sensor/ecg"
TOPIC_SENSOR_MAX = b"iot/sensor/max30102"
TOPIC_SENSOR_MLX = b"iot/sensor/mlx90614"
TOPIC_SENSOR_ULTRASONICO = b"iot/sensor/ultrasonico"
TOPIC_STATUS_MOTORES = b"iot/status/motores"
TOPIC_STATUS_SERVO = b"iot/status/servo"
TOPIC_STATUS_SEGUIMIENTO = b"iot/status/seguimiento"  # Nuevo topic para estado de seguimiento

# Pines motores
in1 = Pin(5, Pin.OUT); in2 = Pin(18, Pin.OUT); ena = PWM(Pin(4), freq=1000); ena.duty(0)
in3 = Pin(15, Pin.OUT); in4 = Pin(19, Pin.OUT); enb = PWM(Pin(2), freq=1000); enb.duty(0)

# Pin servo (GPIO25)
servo_pin = Pin(25, Pin.OUT)
servo = PWM(servo_pin, freq=50)
servo.duty(0)

# Pines ultrasonico
TRIG_PIN = 33
ECHO_PIN = 32
trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

# Estados
aht21 = None
ens160 = None
monitoreando_aire = False
monitoreando_ecg = False
monitoreando_max = False
monitoreando_mlx = False
monitoreando_ultrasonico = False
seguimiento_activo = False  # Nuevo estado para seguimiento
oled = None
client = None

# Configuración seguimiento
DISTANCIA_OBJETIVO = 20  # cm
MARGEN_SEGUIMIENTO = 5   # cm
VELOCIDAD_SEGUIMIENTO = 70  # %

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(SSID, PASSWORD)
        for _ in range(20):
            if wlan.isconnected(): break
            time.sleep(0.5)
    return wlan.isconnected()

def init_oled():
    i2c = I2C(1, scl=Pin(22), sda=Pin(21))
    oled = SSD1306_I2C(128, 64, i2c)
    oled.fill(0)
    oled.text("Iniciando...", 0, 0)
    oled.show()
    return oled

def mostrar_en_oled(*lineas):
    global oled
    if oled is None:
        return
    oled.fill(0)
    for i, linea in enumerate(lineas[:6]):
        oled.text(str(linea), 0, i*10)
    oled.show()

def enviar_a_firebase(ruta, datos):
    try:
        url = f"{FIREBASE_URL}/{ruta}.json?auth={FIREBASE_SECRET}"
        res = urequests.put(url, json=datos)
        res.close()
    except Exception as e:
        print("❌ Error Firebase:", e)

# ... (las funciones existentes clasificar_aqi, enviar_datos_sensor, control_servo, detener_servo, 
# monitorear_ecg, monitorear_max, monitorear_mlx90614 se mantienen igual) ..
def clasificar_aqi(aqi):
    if aqi <= 50: return "Excelente"
    elif aqi <= 100: return "Aceptable"
    elif aqi <= 150: return "Moderado"
    else: return "Malo"

def enviar_datos_sensor():
    global ens160, aht21
    temp, hum = aht21.read()
    aqi, tvoc, eco2 = ens160.read_data()
    if aqi is not None:
        data = {
            "temperatura": temp,
            "humedad": hum,
            "aqi": aqi,
            "tvoc": tvoc,
            "eco2": eco2,
            "clasificacion": clasificar_aqi(aqi),
            "timestamp": time.time()
        }
        client.publish(TOPIC_SENSOR_DATA, json.dumps(data))
        enviar_a_firebase("aire/data", data)

# Reemplaza la función control_servo con esta versión calibrada
def control_servo(angulo):
    """Controla el servomotor con calibración para 0°-180°"""
    if 0 <= angulo <= 180:
        # Valores calibrados (ajusta estos según tu servo)
        MIN_DUTY = 25    # Valor para 0° (ajustar)
        MAX_DUTY = 130    # Valor para 180° (ajustar)
        
        # Mapear ángulo a duty cycle
        duty = int(MIN_DUTY + (angulo * (MAX_DUTY - MIN_DUTY) / 180))
        servo.duty(duty)
        
        estado = {
            "status": "active",
            "angle": angulo,
            "duty": duty,  # Para diagnóstico
            "timestamp": time.time()
        }
        client.publish(TOPIC_STATUS_SERVO, json.dumps(estado))
        mostrar_en_oled(f"Servo: {angulo}°", f"Duty: {duty}", "", "Estado: Activo", "", "")
        return True
    return False
def detener_servo():
    """Detiene el servomotor"""
    servo.duty(0)
    estado = {"status": "inactive", "angle": 0, "timestamp": time.time()}
    client.publish(TOPIC_STATUS_SERVO, json.dumps(estado))
    enviar_a_firebase("servo/status", estado)
    mostrar_en_oled("Servo Control", "Servo detenido", "", "Estado: Inactivo", "", "")

def monitorear_ecg():
    global monitoreando_ecg
    monitoreando_ecg = True
    ecg_pin = ADC(Pin(34))
    ecg_pin.atten(ADC.ATTN_11DB)
    ecg_pin.width(ADC.WIDTH_12BIT)

    umbral = 1500  # ← Ajusta según tu señal real
    estado_anterior = 0
    contador_latidos = 0
    ultimo_envio = time.ticks_ms()
    intervalo_envio = 5000  # cada 5 segundos
    tiempo_refractario = 300  # ms para evitar doble conteo
    ultimo_latido = 0

    def clasificar_ecg(v):
        if v < 1000:
            return "Reposo"
        elif v < 2500:
            return "Moderado"
        else:
            return "Posible latido"

    while monitoreando_ecg:
        valor = ecg_pin.read()
        estado = clasificar_ecg(valor)
        tiempo_actual = time.ticks_ms()

        # Detectar latido solo si ha pasado el tiempo refractario
        if valor > umbral and estado_anterior == 0 and time.ticks_diff(tiempo_actual, ultimo_latido) > tiempo_refractario:
            contador_latidos += 1
            estado_anterior = 1
            ultimo_latido = tiempo_actual
        elif valor < umbral:
            estado_anterior = 0

        # Enviar datos cada 5 segundos
        if time.ticks_diff(tiempo_actual, ultimo_envio) >= intervalo_envio:
            bpm = contador_latidos * (60000 / intervalo_envio)
            datos = {
                "valor": valor,
                "estado": estado,
                "bpm": round(bpm),
                "latidos": contador_latidos,
                "timestamp": time.time()
            }

            # Enviar por MQTT y Firebase
            client.publish(TOPIC_SENSOR_ECG, json.dumps(datos))
            enviar_a_firebase("ecg/data", datos)

            # Mostrar en OLED
            mostrar_en_oled("ECG Monitor"  , f"BPM: {round(bpm)}", f"Estado: {estado}", f"Valor: {valor}", "", "Enviando datos...")

            # Reset
            ultimo_envio = tiempo_actual
            contador_latidos = 0

        time.sleep(0.05)


def monitorear_max():
    global monitoreando_max
    monitoreando_max = True
    i2c = I2C(1, scl=Pin(22), sda=Pin(21))
    time.sleep(1.5)
    sensor = MAX30102(i2c)
    last_beat_time = 0
    bpm_list = []
    THRESHOLD = 1000
    ir_buffer = []
    red_buffer = []

    while monitoreando_max:
        firebase_data = {
            "ir": 0, "red": 0,
            "estado_pulso": "No disponible",
            "estado_oxigeno": "No disponible",
            "timestamp": int(time.time()),
            "bpm": -1, "spo2": -1
        }
        data = sensor.read_fifo()
        if data is None or len(data) != 2:
            enviar_a_firebase("max30102/data", firebase_data)
            time.sleep(0.2)
            continue
        ir, red = data
        firebase_data.update({
            "ir": int(ir), "red": int(red),
            "estado_pulso": "Bajo" if ir < 30000 else "Normal" if ir < 120000 else "Alto",
            "estado_oxigeno": "Bajo" if red < 0 else "Normal" if red < 100 else "Alto"
        })
        ir_buffer.append(ir); red_buffer.append(red)
        if len(ir_buffer) > 100: ir_buffer.pop(0)
        if len(red_buffer) > 100: red_buffer.pop(0)
        ir_dc = sum(ir_buffer)/len(ir_buffer); red_dc = sum(red_buffer)/len(red_buffer)
        ir_ac = ir - ir_dc; red_ac = red - red_dc
        if ir_ac > THRESHOLD:
            current_time = time.ticks_ms()
            if last_beat_time > 0:
                delta = time.ticks_diff(current_time, last_beat_time) / 1000
                if 0.3 < delta < 2.0:
                    bpm = 60 / delta
                    bpm_list.append(bpm)
                    if len(bpm_list) > 5: bpm_list.pop(0)
                    firebase_data["bpm"] = int(sum(bpm_list)/len(bpm_list))
            last_beat_time = current_time
        if red_ac != 0 and red_dc != 0:
            ratio = (ir_ac/ir_dc) / (red_ac/red_dc)
            spO2 = 104 - (17 * ratio)
            firebase_data["spo2"] = int(max(0, min(100, spO2)))
        
        mostrar_en_oled(
            "OXIMETRO MAX30102",
            f"BPM: {firebase_data['bpm']}",
            f"SpO2: {firebase_data['spo2']}%",
            f"IR: {firebase_data['ir']}",
            f"RED: {firebase_data['red']}",
            "Enviando datos..."
        )
        
        client.publish(TOPIC_SENSOR_MAX, json.dumps(firebase_data))
        enviar_a_firebase("max30102/data", firebase_data)
        time.sleep(0.2)
#para ver los datos de las temperatura
def monitorear_mlx90614():
    global monitoreando_mlx
    monitoreando_mlx = True
    i2c_bus = I2C(0, scl=Pin(27), sda=Pin(26), freq=100000)
    MLX_ADDR = 0x5A
    AMBIENT_REG = 0x06
    OBJECT_REG = 0x07

    def leer_temp(reg):
        try:
            data = i2c_bus.readfrom_mem(MLX_ADDR, reg, 3)
            raw = struct.unpack('<H', data[0:2])[0]
            temp_c = (raw * 0.02) - 273.15
            return temp_c
        except Exception as e:
            print("⚠ Error leyendo MLX90614:", e)
            return None

    while monitoreando_mlx:
        t_amb = leer_temp(AMBIENT_REG)
        t_obj = leer_temp(OBJECT_REG)

        if t_amb is not None and t_obj is not None:
            data = {
                "temp_ambiente": t_amb,
                "temp_objeto": t_obj,
                "timestamp": time.time()
            }
            client.publish(TOPIC_SENSOR_MLX, json.dumps(data))
            enviar_a_firebase("mlx90614/data", data)
            
            mostrar_en_oled(
                "TERMOMETRO MLX90614",
                f"Ambiente: {t_amb:.1f}°C",
                f"Objeto: {t_obj:.1f}°C",
                "",
                "Enviando datos...",
                ""
            )
        else:
            mostrar_en_oled("MLX90614", "Error:", "No se pudo leer", "temperatura", "", "Reintentando...")
        
        time.sleep(2)
#nos permite medir la distancia
def medir_distancia():
    trig.off()
    time.sleep_us(5)
    trig.on()
    time.sleep_us(10)
    trig.off()
    dur = time_pulse_us(echo, 1, 30000)
    return (dur * 0.0343) / 2 if dur > 0 else None
#para verificar el ultrasonico que nos permitira ver los datos del ultrasonico
def monitorear_ultrasonico():
    global monitoreando_ultrasonico
    monitoreando_ultrasonico = True
    intervalo_medicion = 1.0
    
    print("🔊 Iniciando monitoreo ultrasónico...")
    mostrar_en_oled("Ultrasonico", "Iniciado", "Midiendo...", "", "", "")
    
    while monitoreando_ultrasonico:
        distancia = medir_distancia()
        
        if distancia is not None:
            print(f"Distancia: {distancia:.2f} cm")
            
            data = {
                "distancia": distancia,
                "timestamp": time.time()
            }
            client.publish(TOPIC_SENSOR_ULTRASONICO, json.dumps(data))
            enviar_a_firebase("ultrasonico/data", data)
            
            mostrar_en_oled("Ultrasonico:", f"Distancia: {distancia:.1f} cm", "", "", "Enviando datos...", "")
        else:
            print("⚠ Sin lectura válida")
            mostrar_en_oled("Ultrasonico:", "Error:", "No se pudo medir", "distancia", "", "Reintentando...")
        
        time.sleep(intervalo_medicion)
    
    print("🛑 Monitoreo ultrasónico detenido")
    mostrar_en_oled("Ultrasonico", "detenido", "Regresando", "al menu...", "", "")
#este es un control para manejar el robot
def control_motores(direction, speed=100):
    speed = max(0, min(100, int(speed)))
    pwm = int(1023 * speed / 100)
    in1.value(0); in2.value(0); in3.value(0); in4.value(0); ena.duty(0); enb.duty(0)
    if direction == "forward": in1.value(1); in3.value(1)
    elif direction == "backward": in2.value(1); in4.value(1)
    elif direction == "left": in1.value(1); in4.value(1)
    elif direction == "right": in2.value(1); in3.value(1)
    if direction != "stop": ena.duty(pwm); enb.duty(pwm)
    client.publish(TOPIC_STATUS_MOTORES, json.dumps({"status": direction, "speed": speed}))
#metodo para el sensor ultrasonic
def seguir_objeto():
    global seguimiento_activo
    seguimiento_activo = True
    
    print("🔄 Iniciando modo seguimiento de objetos...")
    mostrar_en_oled("Modo Seguimiento", "Iniciado", f"Objetivo: {DISTANCIA_OBJETIVO}cm", f"Margen: ±{MARGEN_SEGUIMIENTO}cm", f"Velocidad: {VELOCIDAD_SEGUIMIENTO}%", "")
    
    # Enviar estado inicial a Firebase
    estado = {
        "status": "active",
        "target_distance": DISTANCIA_OBJETIVO,
        "margin": MARGEN_SEGUIMIENTO,
        "speed": VELOCIDAD_SEGUIMIENTO,
        "timestamp": time.time()
    }
    client.publish(TOPIC_STATUS_SEGUIMIENTO, json.dumps(estado))
    enviar_a_firebase("seguimiento/status", estado)
    
    while seguimiento_activo:
        distancia = medir_distancia()
        
        if distancia is None:
            time.sleep(0.1)
            continue
            
        # Lógica de seguimiento
        if distancia < (DISTANCIA_OBJETIVO - MARGEN_SEGUIMIENTO):
            # Objeto demasiado cerca - retroceder
            control_motores("backward", VELOCIDAD_SEGUIMIENTO)
            estado_movimiento = "Retrocediendo"
        elif distancia > (DISTANCIA_OBJETIVO + MARGEN_SEGUIMIENTO):
            # Objeto demasiado lejos - avanzar
            control_motores("forward", VELOCIDAD_SEGUIMIENTO)
            estado_movimiento = "Avanzando"
        else:
            # Distancia correcta - detenerse
            control_motores("stop")
            estado_movimiento = "Manteniendo distancia"
        
        # Actualizar pantalla y enviar datos
        mostrar_en_oled(
            "Modo Seguimiento",
            f"Distancia: {distancia:.1f} cm",
            f"Objetivo: {DISTANCIA_OBJETIVO} cm",
            estado_movimiento,
            f"Velocidad: {VELOCIDAD_SEGUIMIENTO}%",
            "Enviando datos..."
        )
        
        # Enviar datos de seguimiento
        datos = {
            "distancia_actual": distancia,
            "estado": estado_movimiento.lower(),
            "timestamp": time.time()
        }
        client.publish(TOPIC_SENSOR_ULTRASONICO, json.dumps(datos))
        enviar_a_firebase("seguimiento/data", datos)
        
        time.sleep(0.2)
    
    # Al salir del bucle (seguimiento desactivado)
    control_motores("stop")
    estado_final = {
        "status": "inactive",
        "timestamp": time.time()
    }
    client.publish(TOPIC_STATUS_SEGUIMIENTO, json.dumps(estado_final))
    enviar_a_firebase("seguimiento/status", estado_final)
    mostrar_en_oled("Modo Seguimiento", "Detenido", "", "Motores apagados", "", "")
#llama al MQTT para funcionar
def mqtt_callback(topic, msg):
    global monitoreando_aire, monitoreando_ecg, monitoreando_max, monitoreando_mlx
    global monitoreando_ultrasonico, seguimiento_activo, DISTANCIA_OBJETIVO, MARGEN_SEGUIMIENTO, VELOCIDAD_SEGUIMIENTO
    
    topic = topic.decode()
    text = msg.decode().lower()
    
    if topic == TOPIC_CONTROL_MOTORES.decode():
        data = json.loads(text)
        control_motores(data['command'], data.get('speed', 50))
    elif topic == TOPIC_CONTROL_AIRE.decode():
        monitoreando_aire = (text == "start")
    elif topic == TOPIC_CONTROL_ECG.decode():
        if text == "start" and not monitoreando_ecg:
            _thread.start_new_thread(monitorear_ecg, ())
        elif text == "stop":
            monitoreando_ecg = False
    elif topic == TOPIC_CONTROL_MAX.decode():
        if text == "start" and not monitoreando_max:
            _thread.start_new_thread(monitorear_max, ())
        elif text == "stop":
            monitoreando_max = False
    elif topic == TOPIC_CONTROL_MLX.decode():
        if text == "start" and not monitoreando_mlx:
            _thread.start_new_thread(monitorear_mlx90614, ())
        elif text == "stop":
            monitoreando_mlx = False
    elif topic == TOPIC_CONTROL_ULTRASONICO.decode():
        if text == "start" and not monitoreando_ultrasonico:
            _thread.start_new_thread(monitorear_ultrasonico, ())
        elif text == "stop":
            monitoreando_ultrasonico = False
    elif topic == TOPIC_CONTROL_SERVO.decode():
        try:
            if msg == b"stop":
                detener_servo()
            else:
                data = json.loads(msg.decode())
                if 'angle' in data:
                    control_servo(data['angle'])
        except Exception as e:
            print("Error comando servo:", e)
    elif topic == TOPIC_CONTROL_SEGUIMIENTO.decode():  # Manejo del nuevo topic
        try:
            if text == "start":
                if not seguimiento_activo:
                    # Opcional: permitir configuración desde MQTT
                    data = json.loads(msg.decode())
                    if 'target' in data:
                        DISTANCIA_OBJETIVO = float(data['target'])
                    if 'margin' in data:
                        MARGEN_SEGUIMIENTO = float(data['margin'])
                    if 'speed' in data:
                        VELOCIDAD_SEGUIMIENTO = int(data['speed'])
                    
                    _thread.start_new_thread(seguir_objeto, ())
            elif text == "stop":
                seguimiento_activo = False
        except Exception as e:
            print("Error comando seguimiento:", e)
#main que nos permitira ejecutarlo
def main():
    global client, ens160, aht21, oled
    if not connect_wifi():
        time.sleep(5); reset()
    
    try:
        oled = init_oled()
    except Exception as e:
        print("❌ Error OLED:", e)
        oled = None
    
    i2c = I2C(0, scl=Pin(27), sda=Pin(26))
    ens160 = ENS160(i2c); aht21 = AHT21(i2c)
    
    client = MQTTClient(CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
    client.set_callback(mqtt_callback)
    client.connect()
    
    # Suscripciones a topics
    client.subscribe(TOPIC_CONTROL_MOTORES)
    client.subscribe(TOPIC_CONTROL_AIRE)
    client.subscribe(TOPIC_CONTROL_ECG)
    client.subscribe(TOPIC_CONTROL_MAX)
    client.subscribe(TOPIC_CONTROL_MLX)
    client.subscribe(TOPIC_CONTROL_ULTRASONICO)
    client.subscribe(TOPIC_CONTROL_SERVO)
    client.subscribe(TOPIC_CONTROL_SEGUIMIENTO)  # Nueva suscripción
    
    mostrar_en_oled("Sistema listo", "Conectado a MQTT", "Esperando", "comandos...", "", "")
    print("✅ MQTT conectado y suscrito.")
    
    while True:
        client.check_msg()
        if monitoreando_aire: 
            enviar_datos_sensor()
            time.sleep(2)
        time.sleep(0.1)
#ejecucion
if __name__ == "__main__":
    main()
