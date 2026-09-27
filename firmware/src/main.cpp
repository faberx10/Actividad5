/*
 * Actividad 5 - Control de un brazo robótico (URDF) con ESP32 y Python
 * Micros y Laboratorio - UMNG
 *
 * El ESP32 lee 3 potenciómetros y 2 pulsadores y envía una trama por
 * USB-Serial (UART0, 115200 baudios) cada 20 ms (50 Hz):
 *
 *   $<seq>,<t_ms>,<pot_base>,<pot_codo>,<pot_pinza>,<pinza>,<rutina>*<CS>\r\n
 *
 *   seq        contador de tramas (0..65535) -> detectar tramas perdidas
 *   t_ms       millis() del ESP32 al tomar la muestra
 *   pot_*      lectura ADC filtrada, 12 bits (0..4095)
 *   pinza      1 = dedos abiertos, 0 = cerrados (se alterna con el pulsador)
 *   rutina     contador de pulsaciones del botón de rutina automática
 *   CS         checksum XOR (hex) de todos los caracteres entre '$' y '*'
 *
 * Además responde al comando "P<n>\n" del PC con "#PONG,<n>" para que el
 * PC mida el tiempo de ida y vuelta (RTT) de la comunicación.
 */

#include <Arduino.h>
#include "esp_timer.h"

// ---------------- Pines (ESP32 DevKit 30 pines) ----------------
// Potenciómetros en ADC1 (el ADC2 se bloquea si algún día se usa WiFi)
const uint8_t PIN_POT_BASE  = 34;  // joint_1       (giro de la base)
const uint8_t PIN_POT_CODO  = 35;  // joint_2       (codo)
const uint8_t PIN_POT_PINZA = 32;  // joint_gripper (subir / bajar pinza)
// Pulsadores de 2 patas: una pata al GPIO y la otra a GND (pull-up interno)
const uint8_t PIN_BTN_PINZA  = 18;  // abre / cierra los dedos
const uint8_t PIN_BTN_RUTINA = 19;  // lanza la rutina automática de prueba

// ---------------- Parámetros ----------------
const uint32_t PERIODO_US   = 20000;  // 20 ms -> 50 Hz de envío
const uint8_t  MUESTRAS_ADC = 16;     // promedio por lectura (reduce ruido)
const float    ALFA_EMA     = 0.35f;  // filtro exponencial (0..1)
const uint32_t REBOTE_MS    = 30;     // antirrebote de los pulsadores

// ---------------- Temporizador de muestreo ----------------
volatile bool tocaMuestrear = false;
esp_timer_handle_t timerMuestreo;

void alVencerTimer(void *) { tocaMuestrear = true; }  // cada 20 ms

// ---------------- Potenciómetros ----------------
float filtro[3] = {0, 0, 0};
const uint8_t PINES_POT[3] = {PIN_POT_BASE, PIN_POT_CODO, PIN_POT_PINZA};

uint16_t leerPromedio(uint8_t pin) {
  uint32_t suma = 0;
  for (uint8_t i = 0; i < MUESTRAS_ADC; i++) suma += analogRead(pin);
  return suma / MUESTRAS_ADC;
}

// ---------------- Pulsadores con antirrebote ----------------
struct Boton {
  uint8_t pin;
  bool estadoEstable;     // HIGH = suelto (pull-up)
  bool lecturaAnterior;
  uint32_t tCambio;
};
Boton btnPinza  = {PIN_BTN_PINZA,  HIGH, HIGH, 0};
Boton btnRutina = {PIN_BTN_RUTINA, HIGH, HIGH, 0};

// Devuelve true solo en el instante en que se presiona (flanco de bajada)
bool fuePresionado(Boton &b) {
  bool lectura = digitalRead(b.pin);
  if (lectura != b.lecturaAnterior) { b.tCambio = millis(); b.lecturaAnterior = lectura; }
  if ((millis() - b.tCambio) > REBOTE_MS && lectura != b.estadoEstable) {
    b.estadoEstable = lectura;
    if (lectura == LOW) return true;
  }
  return false;
}

bool pinzaAbierta = true;
uint16_t contadorRutina = 0;
uint16_t secuencia = 0;

// ---------------- Comandos del PC (medición de RTT) ----------------
char bufRx[24];
uint8_t idxRx = 0;

void atenderSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      bufRx[idxRx] = '\0';
      if (idxRx > 1 && bufRx[0] == 'P') {
        Serial.printf("#PONG,%s\r\n", bufRx + 1);  // eco inmediato
      }
      idxRx = 0;
    } else if (idxRx < sizeof(bufRx) - 1) {
      bufRx[idxRx++] = c;
    }
  }
}

// ---------------- Envío de la trama ----------------
void enviarTrama() {
  char datos[64];
  int n = snprintf(datos, sizeof(datos), "%u,%lu,%u,%u,%u,%u,%u",
                   secuencia, (unsigned long)millis(),
                   (uint16_t)filtro[0], (uint16_t)filtro[1], (uint16_t)filtro[2],
                   pinzaAbierta ? 1 : 0, contadorRutina);
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)datos[i];
  Serial.printf("$%s*%02X\r\n", datos, cs);
  secuencia++;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // 0..4095

  pinMode(PIN_BTN_PINZA, INPUT_PULLUP);
  pinMode(PIN_BTN_RUTINA, INPUT_PULLUP);

  for (uint8_t i = 0; i < 3; i++) filtro[i] = leerPromedio(PINES_POT[i]);

  // Temporizador periódico de hardware (esp_timer) a 50 Hz
  esp_timer_create_args_t args = {};
  args.callback = &alVencerTimer;
  args.name = "muestreo";
  esp_timer_create(&args, &timerMuestreo);
  esp_timer_start_periodic(timerMuestreo, PERIODO_US);

  Serial.println("#INFO Actividad 5 - Brazo URDF con ESP32");
  Serial.println("#INFO Trama: $seq,t_ms,pot_base,pot_codo,pot_pinza,pinza,rutina*CS a 50 Hz");
}

void loop() {
  // Los pulsadores se revisan en cada vuelta para no perder pulsaciones
  if (fuePresionado(btnPinza)) pinzaAbierta = !pinzaAbierta;
  if (fuePresionado(btnRutina)) contadorRutina++;

  atenderSerial();

  if (tocaMuestrear) {
    tocaMuestrear = false;
    for (uint8_t i = 0; i < 3; i++) {
      filtro[i] = ALFA_EMA * leerPromedio(PINES_POT[i]) + (1.0f - ALFA_EMA) * filtro[i];
    }
    enviarTrama();
  }
}
