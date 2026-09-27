"""
Actividad 5 - Análisis de la comunicación en tiempo real
Lee un registro de logs/ (por defecto el más reciente) y calcula:
  - frecuencia real de las tramas y jitter (variación del intervalo)
  - tramas perdidas y tramas con checksum inválido
  - RTT (tiempo de ida y vuelta PC -> ESP32 -> PC)
  - seguimiento de las articulaciones (objetivo vs posición real)
y guarda dos figuras PNG junto al registro.

Uso:
    python analizar_log.py                       # último registro
    python analizar_log.py logs/sesion_XXXX.csv  # uno en particular
"""

import csv
import glob
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AQUI = os.path.dirname(os.path.abspath(__file__))


def percentil(datos, q):
    datos = sorted(datos)
    k = (len(datos) - 1) * q / 100
    i = int(k)
    return datos[i] if i + 1 >= len(datos) else datos[i] + (datos[i + 1] - datos[i]) * (k - i)


def main():
    if len(sys.argv) > 1:
        ruta = sys.argv[1]
    else:
        registros = sorted(glob.glob(os.path.join(AQUI, "logs", "sesion_*.csv")))
        if not registros:
            print("No hay registros en logs/. Ejecuta primero brazo_control.py con el ESP32.")
            return
        ruta = registros[-1]

    with open(ruta, newline="") as f:
        filas = list(csv.DictReader(f))
    if len(filas) < 10:
        print("El registro tiene muy pocas muestras.")
        return

    col = lambda k: [float(r[k]) for r in filas]
    t = col("t_pc_s")
    seq = [int(r["seq"]) for r in filas]
    t_esp = col("t_esp_ms")
    d_seq = [(b - a) % 65536 for a, b in zip(seq, seq[1:])]
    # Intervalo real de llegada: solo entre tramas consecutivas (d_seq == 1)
    intervalos_pc = [(t[i + 1] - t[i]) * 1000 for i, d in enumerate(d_seq) if d == 1]
    # Periodo de muestreo en el ESP32: tiempo del ESP32 / tramas enviadas
    periodo_esp = [(t_esp[i + 1] - t_esp[i]) / d for i, d in enumerate(d_seq) if d >= 1]
    rtt = [float(r["rtt_ms"]) for r in filas if r["rtt_ms"]]
    t_rtt = [float(r["t_pc_s"]) for r in filas if r["rtt_ms"]]
    primera, ultima = filas[0], filas[-1]
    duracion = t[-1] - t[0]

    ok, malas, perdidas = int(ultima["tramas_ok"]), int(ultima["tramas_malas"]), int(ultima["tramas_perdidas"])
    recibidas_en_registro = ok - int(primera["tramas_ok"]) + 1
    esperadas = ok + perdidas
    rtt_estable = rtt[1:] if len(rtt) > 1 else rtt   # la 1.a se mide al abrir el puerto

    print(f"Registro: {os.path.basename(ruta)}")
    print(f"Duración: {duracion:.1f} s  |  tramas válidas: {ok}  |  filas del registro: {len(filas)}")
    if len(filas) < recibidas_en_registro:
        print(f"  (el registro guardó {len(filas)} de {recibidas_en_registro} tramas; "
              "las demás se recibieron pero no se escribieron)")
    print(f"Frecuencia de recepción: {(recibidas_en_registro - 1) / duracion:.2f} Hz (esperada 50 Hz)")
    print(f"Periodo de muestreo del ESP32: media {statistics.mean(periodo_esp):.2f} ms, "
          f"mín {min(periodo_esp):.2f} ms, máx {max(periodo_esp):.2f} ms")
    print(f"Intervalo de llegada al PC:    media {statistics.mean(intervalos_pc):.2f} ms, "
          f"jitter (desv. est.) {statistics.pstdev(intervalos_pc):.2f} ms, "
          f"p95 {percentil(intervalos_pc, 95):.2f} ms, máx {max(intervalos_pc):.2f} ms")
    print(f"Tramas perdidas: {perdidas} de {esperadas} ({100 * perdidas / max(esperadas, 1):.3f} %)"
          f"  |  checksum inválido: {malas}")
    if rtt:
        print(f"RTT (sin la 1.a medición): mediana {statistics.median(rtt_estable):.2f} ms, "
              f"media {statistics.mean(rtt_estable):.2f} ms, mín {min(rtt_estable):.2f} ms, "
              f"p95 {percentil(rtt_estable, 95):.2f} ms, máx {max(rtt_estable):.2f} ms "
              f"({len(rtt_estable)} mediciones; 1.a al abrir el puerto: {rtt[0]:.1f} ms)")

    base = os.path.splitext(ruta)[0]

    # Figura 1: comunicación
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    ax[0].hist(intervalos_pc, bins=60, color="#2a78c2")
    ax[0].axvline(20, color="#d62728", ls="--", label="20 ms (50 Hz)")
    ax[0].set_title("Intervalo de llegada entre tramas consecutivas (PC)")
    ax[0].set_xlabel("ms")
    ax[0].set_ylabel("cantidad de tramas")
    ax[0].legend()
    if len(rtt) > 1:
        ax[1].plot(t_rtt[1:], rtt_estable, "o-", color="#2ca02c", ms=3)
        ax[1].axhline(statistics.median(rtt_estable), color="#555", ls="--",
                      label=f"mediana {statistics.median(rtt_estable):.1f} ms")
        ax[1].legend()
    ax[1].set_title("RTT PC → ESP32 → PC (sin la medición inicial)")
    ax[1].set_xlabel("tiempo (s)")
    ax[1].set_ylabel("ms")
    fig.suptitle(f"Validación de la comunicación – {ok} tramas, {perdidas} perdidas, {malas} con error")
    fig.tight_layout()
    fig.savefig(base + "_comunicacion.png", dpi=130)

    # Figura 2: seguimiento de articulaciones
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    pares = [("obj_j1", "real_j1", "joint_1 – base (rad)"),
             ("obj_j2", "real_j2", "joint_2 – codo (rad)"),
             ("obj_jg", "real_jg", "joint_gripper – subir pinza (m)"),
             ("obj_dedos", "real_dedos", "dedos – abrir/cerrar (m)")]
    rutina = col("rutina_activa")
    for a, (o, r, titulo) in zip(ax, pares):
        a.plot(t, col(o), color="#d62728", lw=1.2, label="objetivo (sensor)")
        a.plot(t, col(r), color="#1f77b4", lw=1.2, label="posición del robot")
        a.fill_between(t, 0, 1, where=[v > 0 for v in rutina], transform=a.get_xaxis_transform(),
                       color="#ffd54f", alpha=0.3, label="rutina automática")
        a.set_ylabel(titulo, fontsize=9)
        a.grid(alpha=0.3)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[-1].set_xlabel("tiempo (s)")
    fig.suptitle("Seguimiento de las articulaciones del brazo")
    fig.tight_layout()
    fig.savefig(base + "_articulaciones.png", dpi=130)

    print(f"Figuras guardadas:\n  {base}_comunicacion.png\n  {base}_articulaciones.png")


if __name__ == "__main__":
    main()
