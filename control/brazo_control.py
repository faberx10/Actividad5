"""
Actividad 5 - Control de un brazo robótico (URDF) desde un ESP32
Micros y Laboratorio - UMNG

Recibe por UART (USB-Serial) las tramas que envía el ESP32 a 50 Hz:

    $<seq>,<t_ms>,<pot_base>,<pot_codo>,<pot_pinza>,<pinza>,<rutina>*<CS>

y mueve el brazo de brazo.urdf en PyBullet:

    pot_base  -> joint_1        (giro de la base,   -2.5 .. 2.5 rad)
    pot_codo  -> joint_2        (codo,              -2.0 .. 2.0 rad)
    pot_pinza -> joint_gripper  (subir/bajar pinza,  0.0 .. 0.15 m)
    pinza     -> joint_dedo_izq y joint_dedo_der (abrir 0.05 m / cerrar 0.0 m)
    rutina    -> cada pulsación inicia (o cancela) la rutina automática de prueba

También valida la comunicación en tiempo real: checksum de cada trama, tramas
perdidas (número de secuencia), frecuencia de llegada, jitter y RTT (ping-pong
con el ESP32 cada segundo). Todo se guarda en logs/ para analizarlo después
con analizar_log.py.

Uso:
    python brazo_control.py                 # ESP32 en COM5
    python brazo_control.py --port COM7     # otro puerto
    python brazo_control.py --sin-serial    # probar el robot con sliders
"""

import argparse
import csv
import os
import re
import statistics
import threading
import time
from collections import deque

import pybullet as p
import pybullet_data
import serial

AQUI = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(AQUI, "brazo.urdf")
CARPETA_LOGS = os.path.join(AQUI, "logs")

ADC_MIN, ADC_MAX = 60, 4035       # zonas muertas en los extremos del potenciómetro
DEDOS_ABIERTOS, DEDOS_CERRADOS = 0.05, 0.0
TIMEOUT_DATOS = 0.5               # s sin tramas -> "SIN DATOS" (el robot se queda quieto)
DT_SIM = 1.0 / 240.0              # paso de simulación de PyBullet

TRAMA = re.compile(r"^\$([^*]+)\*([0-9A-Fa-f]{2})$")


# --------------------------------------------------------------------------
# Comunicación serial (hilo aparte para no frenar la simulación)
# --------------------------------------------------------------------------
class EnlaceSerial(threading.Thread):
    def __init__(self, puerto, baudios=115200):
        super().__init__(daemon=True)
        self.ser = serial.Serial()
        self.ser.port = puerto
        self.ser.baudrate = baudios
        self.ser.timeout = 0.02
        self.ser.dtr = False          # no reiniciar el ESP32 al abrir el puerto
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self.lock = threading.Lock()
        self.corriendo = True
        self.muestra = None           # última trama válida (dict)
        self.t_muestra = 0.0
        self.tramas_ok = 0
        self.tramas_malas = 0
        self.tramas_perdidas = 0
        self._seq_anterior = None
        self.llegadas = deque(maxlen=100)   # instantes de llegada (s)
        self.rtts = deque(maxlen=60)        # últimos RTT (ms)
        self.rtt_nuevo = None
        self._pings = {}
        self._n_ping = 0

    def run(self):
        buf = b""
        t_ping = 0.0
        while self.corriendo:
            ahora = time.perf_counter()
            if ahora - t_ping >= 1.0:               # ping cada segundo
                self._n_ping += 1
                self._pings[self._n_ping] = time.perf_counter()
                self.ser.write(f"P{self._n_ping}\n".encode())
                t_ping = ahora
            try:
                datos = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                print("Se perdió la conexión con el ESP32.")
                self.corriendo = False
                break
            if not datos:
                continue
            t_llegada = time.perf_counter()
            buf += datos
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                self._procesar(linea.decode(errors="ignore").strip(), t_llegada)

    def _procesar(self, linea, t):
        if not linea:
            return
        if linea.startswith("#PONG,"):
            try:
                n = int(linea[6:])
            except ValueError:
                return
            t_envio = self._pings.pop(n, None)
            if t_envio is not None:
                rtt = (t - t_envio) * 1000.0
                with self.lock:
                    self.rtts.append(rtt)
                    self.rtt_nuevo = rtt
            return
        if linea.startswith("#"):
            print("ESP32:", linea[1:])
            return

        m = TRAMA.match(linea)
        if not m:
            with self.lock:
                self.tramas_malas += 1
            return
        cuerpo, cs_recibido = m.group(1), int(m.group(2), 16)
        cs = 0
        for c in cuerpo:
            cs ^= ord(c)
        try:
            campos = [int(x) for x in cuerpo.split(",")]
        except ValueError:
            campos = []
        if cs != cs_recibido or len(campos) != 7:
            with self.lock:
                self.tramas_malas += 1
            return

        seq, t_ms, pb, pc, pp, pinza, rutina = campos
        with self.lock:
            if self._seq_anterior is not None:
                salto = (seq - self._seq_anterior - 1) % 65536
                if salto < 1000:                   # >1000 = el ESP32 se reinició
                    self.tramas_perdidas += salto
            self._seq_anterior = seq
            self.tramas_ok += 1
            self.llegadas.append(t)
            self.muestra = {"seq": seq, "t_esp": t_ms, "pot_base": pb, "pot_codo": pc,
                            "pot_pinza": pp, "pinza": pinza, "rutina": rutina,
                            "t_llegada": t}
            self.t_muestra = t

    def estadisticas(self):
        with self.lock:
            lleg = list(self.llegadas)
            rtts = list(self.rtts)
            est = {"ok": self.tramas_ok, "malas": self.tramas_malas,
                   "perdidas": self.tramas_perdidas}
        if len(lleg) > 2:
            intervalos = [(b - a) * 1000 for a, b in zip(lleg, lleg[1:])]
            est["hz"] = (len(lleg) - 1) / (lleg[-1] - lleg[0])
            est["jitter"] = statistics.pstdev(intervalos)
        else:
            est["hz"], est["jitter"] = 0.0, 0.0
        est["rtt"] = statistics.mean(rtts) if rtts else float("nan")
        est["rtt_max"] = max(rtts) if rtts else float("nan")
        return est

    def cerrar(self):
        self.corriendo = False
        time.sleep(0.05)
        self.ser.close()


# --------------------------------------------------------------------------
# Rutina automática de prueba (articulaciones y pinza)
# --------------------------------------------------------------------------
# (duración en s, joint_1, joint_2, joint_gripper, dedos)
RUTINA = [
    (2.0,  0.0,  0.0, 0.00, DEDOS_ABIERTOS),   # posición inicial
    (3.0,  2.0,  0.0, 0.00, DEDOS_ABIERTOS),   # base a la izquierda
    (4.0, -2.0,  0.0, 0.00, DEDOS_ABIERTOS),   # base a la derecha
    (2.5,  0.0,  0.0, 0.00, DEDOS_ABIERTOS),   # base al centro
    (2.5,  0.0,  1.2, 0.00, DEDOS_ABIERTOS),   # codo adelante
    (3.5,  0.0, -1.2, 0.00, DEDOS_ABIERTOS),   # codo atrás
    (2.5,  0.0,  0.0, 0.00, DEDOS_ABIERTOS),   # codo al centro
    (1.5,  0.0,  0.0, 0.15, DEDOS_ABIERTOS),   # sube la pinza
    (1.5,  0.0,  0.0, 0.15, DEDOS_CERRADOS),   # cierra
    (1.5,  0.0,  0.0, 0.15, DEDOS_ABIERTOS),   # abre
    (1.5,  0.0,  0.0, 0.00, DEDOS_ABIERTOS),   # baja la pinza
    (2.5,  1.0,  0.8, 0.10, DEDOS_CERRADOS),   # movimiento combinado
    (2.5,  0.0,  0.0, 0.00, DEDOS_ABIERTOS),   # regreso
]
DURACION_RUTINA = sum(paso[0] for paso in RUTINA)


def objetivo_rutina(t):
    acumulado = 0.0
    for dur, j1, j2, jg, dedos in RUTINA:
        acumulado += dur
        if t < acumulado:
            return j1, j2, jg, dedos
    return None


# --------------------------------------------------------------------------
# Utilidades
# --------------------------------------------------------------------------
def adc_a_rango(adc, bajo, alto):
    n = (adc - ADC_MIN) / (ADC_MAX - ADC_MIN)
    n = min(max(n, 0.0), 1.0)
    return bajo + n * (alto - bajo)


def cargar_robot(ventana=True):
    p.connect(p.GUI if ventana else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT_SIM)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF, [0, 0, 0], useFixedBase=True)

    juntas = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        nombre = info[1].decode()
        juntas[nombre] = {"id": i, "min": info[8], "max": info[9],
                          "fuerza": info[10], "vel": info[11],
                          "link": info[12].decode()}
    print("Articulaciones encontradas:")
    for n, j in juntas.items():
        print(f"  [{j['id']}] {n:15s} rango [{j['min']:+.2f}, {j['max']:+.2f}]")

    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-22,
                                 cameraTargetPosition=[0, 0, 0.45])
    return robot, juntas


def mover(robot, juntas, nombre, objetivo):
    j = juntas[nombre]
    objetivo = min(max(objetivo, j["min"]), j["max"])
    p.setJointMotorControl2(robot, j["id"], p.POSITION_CONTROL, targetPosition=objetivo,
                            force=j["fuerza"], maxVelocity=j["vel"])
    return objetivo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sin-serial", action="store_true", help="controlar el robot con sliders")
    ap.add_argument("--sin-ventana", action="store_true", help=argparse.SUPPRESS)  # pruebas automáticas
    ap.add_argument("--duracion", type=float, default=0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    enlace = None
    if not args.sin_serial:
        try:
            enlace = EnlaceSerial(args.port, args.baud)
        except serial.SerialException as e:
            print(f"No se pudo abrir {args.port}: {e}")
            print("Cierra el Serial Monitor de PlatformIO y revisa el puerto.")
            return
        enlace.start()
        print(f"Conectado al ESP32 en {args.port}")

    robot, juntas = cargar_robot(not args.sin_ventana)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1 if args.sin_serial else 0)

    sliders = {}
    if args.sin_serial:
        sliders["joint_1"] = p.addUserDebugParameter("joint_1 (base)", -2.5, 2.5, 0)
        sliders["joint_2"] = p.addUserDebugParameter("joint_2 (codo)", -2.0, 2.0, 0)
        sliders["joint_gripper"] = p.addUserDebugParameter("joint_gripper (subir)", 0, 0.15, 0)
        sliders["pinza"] = p.addUserDebugParameter("pinza abrir=1 / cerrar=0", 0, 1, 1)
        sliders["rutina"] = p.addUserDebugParameter("Rutina automatica", 1, 0, 0)  # botón

    # Registro de la sesión (solo cuando hay ESP32)
    ruta_log, f_log, log = None, None, None
    if enlace:
        os.makedirs(CARPETA_LOGS, exist_ok=True)
        ruta_log = os.path.join(CARPETA_LOGS, time.strftime("sesion_%Y%m%d_%H%M%S.csv"))
        f_log = open(ruta_log, "w", newline="")
        log = csv.writer(f_log)
        log.writerow(["t_pc_s", "seq", "t_esp_ms", "pot_base", "pot_codo", "pot_pinza",
                      "pinza", "rutina_activa", "obj_j1", "obj_j2", "obj_jg", "obj_dedos",
                      "real_j1", "real_j2", "real_jg", "real_dedos",
                      "tramas_ok", "tramas_malas", "tramas_perdidas", "rtt_ms"])

    id_efector = juntas["joint_gripper"]["id"]
    hud_ids = [-1] * 4
    t0 = time.perf_counter()
    t_hud = t_consola = t_traza = 0.0
    pos_anterior = None
    contador_rutina_prev = None
    rutina_inicio = None
    seq_prev = None
    obj = {"joint_1": 0.0, "joint_2": 0.0, "joint_gripper": 0.0, "dedos": DEDOS_ABIERTOS}

    try:
        while p.isConnected():
            ahora = time.perf_counter()
            t = ahora - t0
            if args.duracion and t > args.duracion:
                break

            # ---- 1) Entradas: ESP32 o sliders ----
            muestra, hay_datos = None, True
            if enlace:
                with enlace.lock:
                    muestra = dict(enlace.muestra) if enlace.muestra else None
                    edad = ahora - enlace.t_muestra
                hay_datos = muestra is not None and edad < TIMEOUT_DATOS
                if muestra:
                    if contador_rutina_prev is None:
                        contador_rutina_prev = muestra["rutina"]
                    elif muestra["rutina"] != contador_rutina_prev:      # botón de rutina
                        contador_rutina_prev = muestra["rutina"]
                        rutina_inicio = None if rutina_inicio else t
            else:
                clic = p.readUserDebugParameter(sliders["rutina"])
                if contador_rutina_prev is None:
                    contador_rutina_prev = clic
                elif clic != contador_rutina_prev:
                    contador_rutina_prev = clic
                    rutina_inicio = None if rutina_inicio else t

            # ---- 2) Objetivos de cada articulación ----
            en_rutina = False
            if rutina_inicio is not None:
                r = objetivo_rutina(t - rutina_inicio)
                if r is None:
                    rutina_inicio = None
                else:
                    en_rutina = True
                    obj["joint_1"], obj["joint_2"], obj["joint_gripper"], obj["dedos"] = r

            if not en_rutina:
                if enlace and hay_datos:
                    for nombre, clave in (("joint_1", "pot_base"), ("joint_2", "pot_codo"),
                                          ("joint_gripper", "pot_pinza")):
                        j = juntas[nombre]
                        obj[nombre] = adc_a_rango(muestra[clave], j["min"], j["max"])
                    obj["dedos"] = DEDOS_ABIERTOS if muestra["pinza"] else DEDOS_CERRADOS
                elif not enlace:
                    for nombre in ("joint_1", "joint_2", "joint_gripper"):
                        obj[nombre] = p.readUserDebugParameter(sliders[nombre])
                    abierta = p.readUserDebugParameter(sliders["pinza"]) >= 0.5
                    obj["dedos"] = DEDOS_ABIERTOS if abierta else DEDOS_CERRADOS
                # sin datos del ESP32: se mantienen los últimos objetivos

            # ---- 3) Mandar los objetivos a los motores del simulador ----
            for nombre in ("joint_1", "joint_2", "joint_gripper"):
                mover(robot, juntas, nombre, obj[nombre])
            mover(robot, juntas, "joint_dedo_izq", obj["dedos"])
            mover(robot, juntas, "joint_dedo_der", obj["dedos"])
            p.stepSimulation()

            # ---- 4) Registro (una fila por trama nueva) ----
            if enlace and muestra and muestra["seq"] != seq_prev:
                seq_prev = muestra["seq"]
                reales = [p.getJointState(robot, juntas[n]["id"])[0]
                          for n in ("joint_1", "joint_2", "joint_gripper", "joint_dedo_izq")]
                with enlace.lock:
                    rtt = enlace.rtt_nuevo
                    enlace.rtt_nuevo = None
                    cont = (enlace.tramas_ok, enlace.tramas_malas, enlace.tramas_perdidas)
                log.writerow([f"{muestra['t_llegada'] - t0:.4f}", muestra["seq"], muestra["t_esp"], muestra["pot_base"],
                              muestra["pot_codo"], muestra["pot_pinza"], muestra["pinza"],
                              int(en_rutina), f"{obj['joint_1']:.4f}", f"{obj['joint_2']:.4f}",
                              f"{obj['joint_gripper']:.4f}", f"{obj['dedos']:.4f}",
                              *[f"{v:.4f}" for v in reales], *cont,
                              f"{rtt:.2f}" if rtt is not None else ""])

            # ---- 5) Trayectoria del efector (línea verde que se desvanece) ----
            if t - t_traza > 0.05:
                t_traza = t
                pos = p.getLinkState(robot, id_efector, computeForwardKinematics=True)[4]
                if pos_anterior and sum((a - b) ** 2 for a, b in zip(pos, pos_anterior)) > 2.5e-5:
                    p.addUserDebugLine(pos_anterior, pos, [0, 0.8, 0], 3, lifeTime=4)
                pos_anterior = pos

            # ---- 6) Indicadores en pantalla y consola ----
            if t - t_hud > 0.25:
                t_hud = t
                if enlace:
                    e = enlace.estadisticas()
                    estado = "RUTINA AUTOMATICA" if en_rutina else ("ESP32 OK" if hay_datos else "SIN DATOS")
                    lineas = [
                        (f"Estado: {estado}", [0, 0.5, 0] if hay_datos else [0.8, 0, 0]),
                        (f"Tramas: {e['hz']:.1f} Hz | jitter {e['jitter']:.1f} ms", [0, 0, 0]),
                        (f"RTT: {e['rtt']:.1f} ms (max {e['rtt_max']:.1f})", [0, 0, 0]),
                        (f"OK {e['ok']} | malas {e['malas']} | perdidas {e['perdidas']}", [0, 0, 0]),
                    ]
                else:
                    lineas = [(f"Modo sliders {'- RUTINA' if en_rutina else ''}", [0, 0, 0.6]),
                              ("", [0, 0, 0]), ("", [0, 0, 0]), ("", [0, 0, 0])]
                for k, (texto, color) in enumerate(lineas):
                    hud_ids[k] = p.addUserDebugText(texto, [-0.6, -0.9, 1.25 - 0.08 * k], color,
                                                    textSize=1.3, replaceItemUniqueId=hud_ids[k])

            if enlace and t - t_consola > 1.0:
                t_consola = t
                e = enlace.estadisticas()
                print(f"[{t:6.1f}s] {e['hz']:5.1f} Hz  jitter {e['jitter']:4.1f} ms  "
                      f"RTT {e['rtt']:5.1f} ms  ok {e['ok']}  malas {e['malas']}  "
                      f"perdidas {e['perdidas']}  j1={obj['joint_1']:+.2f} j2={obj['joint_2']:+.2f} "
                      f"jg={obj['joint_gripper']:.3f} dedos={'abiertos' if obj['dedos'] else 'cerrados'}")

            # ---- 7) Mantener la simulación en tiempo real ----
            espera = DT_SIM - (time.perf_counter() - ahora)
            if espera > 0:
                time.sleep(espera)
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        if enlace:
            enlace.cerrar()
            f_log.close()
            print(f"Registro guardado en: {ruta_log}")
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
