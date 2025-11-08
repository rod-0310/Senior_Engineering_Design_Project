#include <Arduino.h>
#include <max6675.h>

/* ------------------ Pines MAX6675 ------------------ */
#define PIN_SO   19
#define PIN_CS    5
#define PIN_SCK  18

/* ------------------ Pines de salida ------------------ */
/* Heater (foco AC con dimmer, control lógico de 3.3V/5V) */
#define PIN_DIMMER_HEATER 21
/* Ventiladores DC (PWM a MOSFET o driver) */
#define PIN_PWM_FAN_IN    22
#define PIN_PWM_FAN_OUT   23

/* ------------------ Configuración PWM ------------------ */
const uint32_t PWM_FREQ_HZ = 1000;   // 1 kHz (ajusta si tu dimmer requiere menos)

/* ------------------ Muestreo ------------------ */
const uint16_t Ts_ms          = 1000;  // muestreo cada 1 s
const uint16_t CONV_MS        = 250;   // conversión del MAX6675
const float    CALIB_OFFSET_C = 0.0;   // corrección de calibración °C

/* ------------------ Ensayos ------------------ */
// Heater (foco)
const uint16_t BASE_HEAT_s = 30;
const uint16_t STEP_HEAT_s = 120;
const uint16_t POST_HEAT_s = 60;
const float    U_HEAT      = 1.00f;   // 🔥 100 % de potencia

// Fan In
const uint16_t BASE_FIN_s  = 30;
const uint16_t STEP_FIN_s  = 120;
const uint16_t POST_FIN_s  = 60;
const float    U_FANIN     = 0.50f;   // 50 % PWM

// Fan Out
const uint16_t BASE_FOUT_s = 30;
const uint16_t STEP_FOUT_s = 120;
const uint16_t POST_FOUT_s = 60;
const float    U_FANOUT    = 0.50f;   // 50 % PWM

/* ------------------ Límites de seguridad ------------------ */
const float TEMP_MAX_C = 45.0;   // Si se supera, apaga todo por seguridad

/* ------------------ Objetos ------------------ */
MAX6675 tc(PIN_SCK, PIN_CS, PIN_SO);

/* ------------------ Estado ------------------ */
unsigned long t0_ms = 0, last_sample = 0;
int phase = 0; // 0=heater, 1=fan_in, 2=fan_out, 3=done

/* ------------------ Funciones auxiliares ------------------ */
inline uint8_t dutyFrom01(float u) {
  if (u < 0) u = 0;
  if (u > 1) u = 1;
  return (uint8_t)roundf(u * 255.0f); // 8-bit por defecto
}

inline void setPWM01(float h, float fi, float fo) {
  analogWrite(PIN_DIMMER_HEATER, dutyFrom01(h));
  analogWrite(PIN_PWM_FAN_IN,    dutyFrom01(fi));
  analogWrite(PIN_PWM_FAN_OUT,   dutyFrom01(fo));
}

float readTempC() {
  float c = tc.readCelsius();
  if (isnan(c)) return NAN;
  return c + CALIB_OFFSET_C;
}

/* ------------------ SETUP ------------------ */
void setup() {
  Serial.begin(115200);
  delay(300);

  // Configurar frecuencia PWM por pin (ESP32 Core 3.x)
  analogWriteFrequency(PIN_DIMMER_HEATER, PWM_FREQ_HZ);
  analogWriteFrequency(PIN_PWM_FAN_IN,    PWM_FREQ_HZ);
  analogWriteFrequency(PIN_PWM_FAN_OUT,   PWM_FREQ_HZ);

  setPWM01(0, 0, 0);  // todo apagado

  Serial.println();
  Serial.println(F("=== ESP32 + MAX6675 + DIMMER AC + 2x FAN DC ==="));
  Serial.println(F("# CSV: t_s,u_heater,u_fan_in,u_fan_out,temp_c"));
  Serial.println(F("# Ensayo 1: HEATER (100%)"));
  Serial.println("t_s,u_heater,u_fan_in,u_fan_out,temp_c");

  t0_ms = millis();
  last_sample = 0;
  phase = 0;
}

/* ------------------ LOOP PRINCIPAL ------------------ */
void loop() {
  unsigned long now = millis();
  float t_s = (now - t0_ms) / 1000.0f;

  // Leer temperatura
  float T = readTempC();
  delay(CONV_MS); // dejar tiempo para próxima conversión

  // Protección por temperatura alta
  if (!isnan(T) && T > TEMP_MAX_C) {
    setPWM01(0, 0, 0);
    Serial.println("# Seguridad: Temperatura alta, todo apagado.");
    delay(5000);
    return; // esperar enfriamiento
  }

  // Muestreo y envío cada Ts_ms
  if (now - last_sample >= Ts_ms) {
    last_sample = now;

    float uH = 0, uFI = 0, uFO = 0;
    switch (phase) {
      case 0: if (t_s >= BASE_HEAT_s && t_s < BASE_HEAT_s + STEP_HEAT_s) uH = U_HEAT; break;
      case 1: if (t_s >= BASE_FIN_s  && t_s < BASE_FIN_s  + STEP_FIN_s ) uFI = U_FANIN; break;
      case 2: if (t_s >= BASE_FOUT_s && t_s < BASE_FOUT_s + STEP_FOUT_s) uFO = U_FANOUT; break;
    }

    Serial.print(t_s, 2); Serial.print(',');
    Serial.print(uH, 3);  Serial.print(',');
    Serial.print(uFI, 3); Serial.print(',');
    Serial.print(uFO, 3); Serial.print(',');
    if (isnan(T)) Serial.println("nan");
    else          Serial.println(T, 3);
  }

  // Máquina de estados (tres ensayos)
  unsigned long elapsed_s = (now - t0_ms) / 1000;

  switch (phase) {
    // -------- HEATER 100% --------
    case 0:
      if (elapsed_s < BASE_HEAT_s) {
        setPWM01(0, 0, 0);
      } else if (elapsed_s < BASE_HEAT_s + STEP_HEAT_s) {
        setPWM01(U_HEAT, 0, 0);  // 🔥 100 %
      } else if (elapsed_s < BASE_HEAT_s + STEP_HEAT_s + POST_HEAT_s) {
        setPWM01(0, 0, 0);
      } else {
        Serial.println(F("# --- Ensayo 2: FAN_IN ---"));
        t0_ms = now;
        phase = 1;
      }
      break;

    // -------- FAN IN --------
    case 1:
      if (elapsed_s < BASE_FIN_s) {
        setPWM01(0, 0, 0);
      } else if (elapsed_s < BASE_FIN_s + STEP_FIN_s) {
        setPWM01(0, U_FANIN, 0);
      } else if (elapsed_s < BASE_FIN_s + STEP_FIN_s + POST_FIN_s) {
        setPWM01(0, 0, 0);
      } else {
        Serial.println(F("# --- Ensayo 3: FAN_OUT ---"));
        t0_ms = now;
        phase = 2;
      }
      break;

    // -------- FAN OUT --------
    case 2:
      if (elapsed_s < BASE_FOUT_s) {
        setPWM01(0, 0, 0);
      } else if (elapsed_s < BASE_FOUT_s + STEP_FOUT_s) {
        setPWM01(0, 0, U_FANOUT);
      } else if (elapsed_s < BASE_FOUT_s + STEP_FOUT_s + POST_FOUT_s) {
        setPWM01(0, 0, 0);
      } else {
        Serial.println(F("# === FIN DE ENSAYOS ==="));
        setPWM01(0, 0, 0);
        phase = 3;
      }
      break;

    default:
      setPWM01(0, 0, 0);
      break;
  }
}
