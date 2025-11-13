#include <SmoothThermistor.h>
#include <PID_v1.h>
#include <esp32-hal-timer.h>
#include <esp32-hal-ledc.h>
#include <driver/adc.h>

/* ===================== CONFIG ===================== */
#define MAINS_FREQ_HZ        50
static const uint32_t HALF_CYCLE_US = (1000000UL / (MAINS_FREQ_HZ * 2));

double SETPOINT_C = 23.0;               // Setpoint inicial
const double SP_MIN = 0.0;              // límites de seguridad para SP
const double SP_MAX = 80.0;

// PID_v1 (continuo) — Ts=0.1 s
double Kp = 1.3450;
double Ki = 1.0849;
double Kd = 0.9846;
const uint16_t PID_SAMPLE_MS = 100;      // 0.1 s

// Salida PID (tus dimmer/triac arrancan desde 25 %)
const double U_MIN = 25.0;   // 25 %
const double U_MAX = 100.0;  // 100 %

const int PIN_FAN_IN   = 12;           // ventilador de inyección
const int PIN_FAN_OUT  = 26;           // ventilador de extracción
const int FAN_PWM_HZ   = 25000;        // si no arrancan, prueba 2000..1000
const int FAN_PWM_BITS = 10;
const uint32_t FAN_MAX = (1u << FAN_PWM_BITS) - 1;

// Ley inversa ventiladores <-> heater
const double FAN_MIN_PCT = 0.0;
const double FAN_MAX_PCT = 100.0;

// Dimmer
const int PIN_ZC    = 27;
const int PIN_DIM   = 14;
const uint32_t GATE_PULSE_US = 75;
const uint32_t ZC_BLANK_US   = 500;   // antirrebotes del ZC (~0.5 ms)

// Termistor NTC
const int PIN_NTC_ADC = 35;
SmoothThermistor ntc(
  PIN_NTC_ADC,
  ADC_SIZE_12_BIT,
  100000,  // R0
  100000,  // R serie
  3950,
  25,
  25
);

const double TEMP_MIN_C = -10.0;
const double TEMP_MAX_C = 120.0;

/* ===================== ESTADO ===================== */
volatile double y_tempC = 25.0;       // variable de proceso (filtrada)
volatile double u_duty  = 0.0;        // % heater (PID)
PID myPID((double*)&y_tempC, (double*)&u_duty, &SETPOINT_C, Kp, Ki, Kd, DIRECT);

volatile bool safety_trip = false;
volatile bool sensor_ok   = true;

volatile double t_raw_last   = NAN;
volatile double fan_pct_last = 0.0;    // promedio de ambos fans (telemetría)
volatile double fan_in_pct   = 0.0;    // solo para CSV
volatile double fan_out_pct  = 0.0;    // solo para CSV

hw_timer_t* fireTimer = nullptr;

// Señales seguras para ISR
volatile uint32_t u_delay_us = 0;     // retardo desde ZC -> disparo
volatile bool gate_enable = true;     // inhibe disparo en safety
volatile uint32_t last_zc_us = 0;

/* ===================== FILTROS ===================== */
#define USE_MEDIAN5       1
#define USE_EMA_IIR       1
#define USE_RATE_LIMITER  1

static const double SENSE_DT_S = 0.150;
static const double TEMP_TAU_S = 1.5;
static const double MAX_SLOPE_C_PER_S = 2.0;

static double ema_state = NAN;
static double last_filtered = NAN;
static double fan_cmd_smooth = NAN;   // suavizado de comando de ventiladores

#if USE_MEDIAN5
static double m5_buf[5];
static uint8_t m5_idx = 0;
static bool m5_filled = false;

double median5_push(double x) {
  m5_buf[m5_idx] = x;
  m5_idx = (m5_idx + 1) % 5;
  if (m5_idx == 0) m5_filled = true;
  double v[5];
  uint8_t n = m5_filled ? 5 : m5_idx;
  for (uint8_t i=0;i<n;i++) v[i] = m5_buf[i];
  for (uint8_t i=1;i<n;i++) {
    double key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) { v[j+1] = v[j]; j--; }
    v[j+1] = key;
  }
  if (n == 0) return x;
  if (n % 2 == 1) return v[n/2];
  return 0.5*(v[n/2 - 1] + v[n/2]);
}
#endif

#if USE_EMA_IIR
double ema_update(double x) {
  const double alpha = SENSE_DT_S / (TEMP_TAU_S + SENSE_DT_S);
  if (isnan(ema_state)) ema_state = x;
  ema_state = ema_state + alpha * (x - ema_state);
  return ema_state;
}
#endif

#if USE_RATE_LIMITER
double slope_limit(double x_prev, double x_now) {
  if (isnan(x_prev)) return x_now;
  const double max_step = MAX_SLOPE_C_PER_S * SENSE_DT_S;
  double delta = x_now - x_prev;
  if (delta >  max_step) return x_prev + max_step;
  if (delta < -max_step) return x_prev - max_step;
  return x_now;
}
#endif

double filter_temperature(double t_raw) {
  double t = t_raw;
  #if USE_MEDIAN5
    t = median5_push(t);
  #endif
  #if USE_EMA_IIR
    t = ema_update(t);
  #endif
  #if USE_RATE_LIMITER
    t = slope_limit(last_filtered, t);
  #endif
  last_filtered = t;
  return t;
}

/* ===================== UTILS ===================== */
inline uint32_t dutyToDelayUs(double dutyPct) {
  // Mapeo fase: 0% => retardo máximo (casi sin potencia), 100% => retardo mínimo
  double d = constrain(dutyPct, 0.0, 100.0);
  double alpha = (1.0 - d/100.0) * PI;   // 0..PI
  uint32_t delayUs = (uint32_t)((alpha/PI) * HALF_CYCLE_US);
  if (delayUs < 200) delayUs = 200;
  if (delayUs > HALF_CYCLE_US - 200) delayUs = HALF_CYCLE_US - 200;
  return delayUs;
}

inline double heater_to_fan_inverse(double heaterPct) {
  // Normaliza heater al rango real [U_MIN..U_MAX] y lo invierte a [FAN_MAX..FAN_MIN]
  double h = constrain(heaterPct, U_MIN, U_MAX);
  double norm = (h - U_MIN) / (U_MAX - U_MIN);  // 0..1
  double fan = FAN_MAX_PCT * (1.0 - norm);      // inverso
  fan = constrain(fan, FAN_MIN_PCT, FAN_MAX_PCT);
  return fan;
}

inline double fan_smooth(double target) {
  const double alpha = 0.2; // más grande = más rápido
  if (isnan(fan_cmd_smooth)) fan_cmd_smooth = target;
  fan_cmd_smooth = fan_cmd_smooth + alpha * (target - fan_cmd_smooth);
  return fan_cmd_smooth;
}

/* ===================== ISRs ===================== */
void IRAM_ATTR onFireTimer() {
  static uint32_t last_fire = 0;
  uint32_t now = micros();
  if (now - last_fire < 2000) { // evita doble disparo en el mismo semiciclo
    timerStop(fireTimer);
    return;
  }
  last_fire = now;

  if (!gate_enable) { // seguridad
    timerStop(fireTimer);
    return;
  }

  digitalWrite(PIN_DIM, HIGH);
  delayMicroseconds(GATE_PULSE_US);
  digitalWrite(PIN_DIM, LOW);
  timerStop(fireTimer);
}

void IRAM_ATTR isrZeroCross() {
  uint32_t now = micros();
  if (now - last_zc_us < ZC_BLANK_US) return; // antirrebotes
  last_zc_us = now;

  if (!gate_enable) return;

  timerStop(fireTimer);
  timerWrite(fireTimer, 0);
  timerAlarm(fireTimer, u_delay_us, false, 0);
  timerStart(fireTimer);
}

/* ===================== TAREAS ===================== */
void taskSense(void* pv) {
  for (;;) {
    double t_raw = ntc.temperature();
    t_raw_last = t_raw;
    bool ok = !isnan(t_raw) && t_raw > TEMP_MIN_C && t_raw < TEMP_MAX_C;
    sensor_ok = ok;
    if (!ok) { safety_trip = true; vTaskDelay(pdMS_TO_TICKS(150)); continue; }
    double t_filt = filter_temperature(t_raw);
    y_tempC = t_filt;
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

void taskControl(void* pv) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(PID_SAMPLE_MS);
  for (;;) {
    vTaskDelayUntil(&last, period);

    // Safety: inhibe triac y pone PID en manual
    if (safety_trip || !sensor_ok) {
      gate_enable = false;               // NO dispares triac
      if (myPID.GetMode() == AUTOMATIC)  // evita windup
        myPID.SetMode(MANUAL);
      u_duty = 0.0;                      // informativo
      u_delay_us = HALF_CYCLE_US;        // un valor seguro
      continue;
    }

    // Si venimos de safety, vuelve a automático
    if (myPID.GetMode() == MANUAL) {
      myPID.SetMode(AUTOMATIC);
    }
    gate_enable = true;

    // Calcula PID y convierte a retardo para ISR
    myPID.Compute();                     // u_duty en [%], limitado por OutputLimits
    u_delay_us = dutyToDelayUs(u_duty);
  }
}

void taskActuators(void* pv) {
  for (;;) {
    double fanPct;

    if (safety_trip || !sensor_ok) {
      // Falla: heater inhibido y ventilación al máximo
      fanPct = FAN_MAX_PCT;
    } else {
      // Ley: fans = inverso del heater (dimmer) en su rango real [U_MIN..U_MAX]
      double heaterPct = u_duty;                         // [%]
      double fanTarget = heater_to_fan_inverse(heaterPct);
      fanPct = fan_smooth(fanTarget);                    // suavizado
    }

    // Aplica al hardware: MISMA SEÑAL a ambos (inyección 25 y extracción 26)
    uint32_t duty_counts = (uint32_t)(fanPct * FAN_MAX / 100.0);
    ledcWrite(PIN_FAN_IN,  duty_counts);
    ledcWrite(PIN_FAN_OUT, duty_counts);

    // Telemetría
    fan_in_pct  = fanPct;
    fan_out_pct = fanPct;
    fan_pct_last = fanPct; // promedio (son iguales)

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(300);

  // ADC config (ESP32 ADC1)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // PWM fans (API por pin)
  ledcAttach(PIN_FAN_IN,  FAN_PWM_HZ, FAN_PWM_BITS);
  ledcAttach(PIN_FAN_OUT, FAN_PWM_HZ, FAN_PWM_BITS);
  ledcWrite(PIN_FAN_IN,  0);
  ledcWrite(PIN_FAN_OUT, 0);

  // Dimmer / ZC
  pinMode(PIN_DIM, OUTPUT);
  digitalWrite(PIN_DIM, LOW);
  pinMode(PIN_ZC, INPUT_PULLUP);

  // Timer de disparo a 1 MHz
  fireTimer = timerBegin(1000000);
  timerAttachInterrupt(fireTimer, &onFireTimer);

  // ZC por ambos flancos + antirrebotes en ISR
  attachInterrupt(digitalPinToInterrupt(PIN_ZC), isrZeroCross, CHANGE);

  // PID config
  myPID.SetOutputLimits(U_MIN, U_MAX);
  myPID.SetSampleTime(PID_SAMPLE_MS);
  myPID.SetMode(AUTOMATIC);

  // Tareas
  xTaskCreatePinnedToCore(taskSense,     "sense",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskControl,   "control", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskActuators, "act",     4096, NULL, 1, NULL, 0);

  Serial.println("ms,T_rawC,T_filtC,u_PID_pct,SP_C,Fan_in_pct,Fan_out_pct,safety,u_delay_us,gate");
  Serial.println("Inicio OK (PID & SP modificables por terminal).");
  Serial.println("Comandos: KP <v> | KI <v> | KD <v> | SP <v> | SP+ <d> | SP- <d> | SHOW | SENS | SAFE ON | SAFE OFF | HELP");
}

/* ===================== LOOP ===================== */
void loop() {
  // CSV para depuración cada 1 s
  static uint32_t t0 = millis();
  if (millis() - t0 > 1000) {
    t0 = millis();
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%u,%d\n",
                  (unsigned long)millis(),
                  t_raw_last,
                  y_tempC,
                  u_duty,
                  SETPOINT_C,
                  fan_in_pct,
                  fan_out_pct,
                  (int)safety_trip,
                  (unsigned)u_delay_us,
                  (int)gate_enable);
  }

  // === Terminal ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    String up = cmd; up.toUpperCase();

    auto clampSP = [&](double sp){
      if (sp < SP_MIN) sp = SP_MIN;
      if (sp > SP_MAX) sp = SP_MAX;
      return sp;
    };

    if (up.startsWith("KP ")) {
      Kp = cmd.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.printf("Nuevo Kp = %.4f\n", Kp);

    } else if (up.startsWith("KI ")) {
      Ki = cmd.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.printf("Nuevo Ki = %.4f\n", Ki);

    } else if (up.startsWith("KD ")) {
      Kd = cmd.substring(3).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.printf("Nuevo Kd = %.4f\n", Kd);

    } else if (up.startsWith("SP+ ")) {
      double d = cmd.substring(4).toFloat();
      SETPOINT_C = clampSP(SETPOINT_C + d);
      Serial.printf("SP aumentado: %.2f °C\n", SETPOINT_C);

    } else if (up.startsWith("SP- ")) {
      double d = cmd.substring(4).toFloat();
      SETPOINT_C = clampSP(SETPOINT_C - d);
      Serial.printf("SP reducido: %.2f °C\n", SETPOINT_C);

    } else if (up.startsWith("SP ")) {
      double sp = cmd.substring(3).toFloat();
      SETPOINT_C = clampSP(sp);
      Serial.printf("Nuevo SP = %.2f °C\n", SETPOINT_C);

    } else if (up == "SHOW") {
      Serial.printf("PID: Kp=%.4f  Ki=%.4f  Kd=%.4f | SP=%.2f °C | T=%.2f °C | U=%.2f%% | gate=%d\n",
                    Kp, Ki, Kd, SETPOINT_C, y_tempC, u_duty, (int)gate_enable);

    } else if (up == "SENS") {
      Serial.printf("T_raw=%.2f  T_filt=%.2f  ok=%d\n", t_raw_last, y_tempC, sensor_ok);

    } else if (up == "SAFE ON") {
      safety_trip = true;  // taskControl se encarga de inhibir gate y PID
      Serial.println("Safety TRIP activado.");

    } else if (up == "SAFE OFF") {
      safety_trip = false; // taskControl restaurará automático
      Serial.println("Safety TRIP liberado.");

    } else if (up == "HELP") {
      Serial.println("Comandos:");
      Serial.println("  KP <v>, KI <v>, KD <v>        -> Cambia ganancias PID");
      Serial.println("  SP <v>                        -> Fija setpoint (0..80 °C)");
      Serial.println("  SP+ <d> / SP- <d>             -> Incrementa/decrementa setpoint");
      Serial.println("  SHOW                          -> Muestra estado actual");
      Serial.println("  SENS                          -> Muestra sensores");
      Serial.println("  SAFE ON / SAFE OFF            -> Fuerza/levanta estado de seguridad");

    } else {
      Serial.println("Comando inválido. Escribe HELP.");
    }
  }

  delay(10);
}