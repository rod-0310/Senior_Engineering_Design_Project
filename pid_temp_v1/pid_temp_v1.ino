/*
  ESP32 core 3.x - Control de temperatura con PID_v1
  - NTC 100k + Rserie 10k (SmoothThermistor)
  - Dimmer AC con cruce por cero (ZERO -> ISR, DIM -> pulso gate)
  - Ventiladores DC por PWM (LEDC 25 kHz con nueva API ledcAttach/ledcWrite)
  - PID_v1 (Kp, Ki, Kd continuos) con SampleTime=100 ms

  ¡PELIGRO!: 110/220 VAC requiere aislamiento y protección.
*/

#include <Arduino.h>
#include <SmoothThermistor.h>
#include <PID_v1.h>
#include <esp32-hal-timer.h>   // API nueva de timers (core 3.x)
#include <esp32-hal-ledc.h>    // API nueva LEDC (core 3.x)

/* ===================== CONFIG ===================== */
#define MAINS_FREQ_HZ        50
static const uint32_t HALF_CYCLE_US = (1000000UL / (MAINS_FREQ_HZ * 2));

double SETPOINT_C = 25.0;

// PID_v1 (continuo) — tus valores y Ts=0.1 s
double Kp = 0.2161;
double Ki = 0.005405;
double Kd = 0.01079;
const uint16_t PID_SAMPLE_MS = 100;      // 0.1 s
const double U_MIN = 0.0;
const double U_MAX = 100.0;

// Ventiladores (nueva API LEDC sin “canales”)
const int PIN_FAN_IN   = 25;
const int PIN_FAN_OUT  = 26;
const int FAN_PWM_HZ   = 25000;
const int FAN_PWM_BITS = 10;
const uint32_t FAN_MAX = (1u << FAN_PWM_BITS) - 1;
double fan_kp_cooling  = 1.2;            // % por °C

// Dimmer (tu módulo 2ch: usamos ZERO y DIM de un canal)
const int PIN_ZC    = 27;                // ZERO (entrada)
const int PIN_DIM   = 14;                // DIM/PSM (salida)
const uint32_t GATE_PULSE_US = 75;

// NTC + SmoothThermistor
const int PIN_NTC_ADC = 34;
SmoothThermistor ntc(
  PIN_NTC_ADC,
  ADC_SIZE_12_BIT,
  100000,  // R0
  10000,   // R serie
  3950,    // Beta
  25,      // °C nominal
  15       // muestras
);

const double TEMP_MIN_C = -10.0;
const double TEMP_MAX_C = 120.0;

/* ===================== ESTADO ===================== */
// PID_v1
double y_tempC = 25.0;      // Input
double u_duty  = 0.0;       // Output (0..100 %)
PID myPID(&y_tempC, &u_duty, &SETPOINT_C, Kp, Ki, Kd, DIRECT);

// Seguridad
volatile bool safety_trip = false;
volatile bool sensor_ok   = true;

// Timer para disparo del triac (API nueva)
hw_timer_t* fireTimer = nullptr;

/* ===================== UTILS ===================== */
inline uint32_t angleToDelayUs(double dutyPct) {
  double d = constrain(dutyPct, 0.0, 100.0);
  double alpha = (1.0 - d/100.0) * PI;
  double frac  = alpha / PI;
  uint32_t delayUs = (uint32_t)(frac * HALF_CYCLE_US);
  if (delayUs < 200) delayUs = 200;
  if (delayUs > HALF_CYCLE_US - 200) delayUs = HALF_CYCLE_US - 200;
  return delayUs;
}

/* ===================== ISRs ===================== */
void IRAM_ATTR onFireTimer() {
  // pulso de gate
  digitalWrite(PIN_DIM, HIGH);
  delayMicroseconds(GATE_PULSE_US);
  digitalWrite(PIN_DIM, LOW);
  // detener one-shot
  timerStop(fireTimer);
}

void IRAM_ATTR isrZeroCross() {
  // programa un one-shot en microsegundos con la nueva API
  uint32_t delayUs = angleToDelayUs(u_duty);
  timerStop(fireTimer);
  timerWrite(fireTimer, 0);                 // reinicia el contador
  timerAlarm(fireTimer, delayUs, false, 0); // one-shot: autoreload=false
  timerStart(fireTimer);
}

/* ===================== TAREAS ===================== */
void taskSense(void* pv) {
  for (;;) {
    double t = ntc.temperature();
    bool ok = !isnan(t) && t > TEMP_MIN_C && t < TEMP_MAX_C;
    y_tempC   = t;
    sensor_ok = ok;
    if (!ok) safety_trip = true;
    vTaskDelay(pdMS_TO_TICKS(150)); // ~6–7 Hz
  }
}

void taskControl(void* pv) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(PID_SAMPLE_MS);
  for (;;) {
    vTaskDelayUntil(&last, period);
    if (safety_trip || !sensor_ok) { u_duty = 0.0; continue; }
    myPID.Compute(); // output: u_duty  (0..100 %)
  }
}

void taskActuators(void* pv) {
  for (;;) {
    // Ventiladores proporcional a sobretemperatura
    double over = y_tempC - SETPOINT_C;
    double fanPct = (over > 0.0) ? (fan_kp_cooling * over) : 0.0;
    if (fanPct > 100.0) fanPct = 100.0;

    uint32_t duty = (uint32_t)(fanPct * FAN_MAX / 100.0);
    ledcWrite(PIN_FAN_IN,  duty);   // nueva API: escribe por PIN
    ledcWrite(PIN_FAN_OUT, duty);

    if (safety_trip || !sensor_ok) {
      u_duty = 0.0;
      ledcWrite(PIN_FAN_IN,  FAN_MAX);
      ledcWrite(PIN_FAN_OUT, FAN_MAX);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(300);

  // LEDC nueva API: se adjunta por PIN (sin canales)
  ledcAttach(PIN_FAN_IN,  FAN_PWM_HZ, FAN_PWM_BITS);
  ledcAttach(PIN_FAN_OUT, FAN_PWM_HZ, FAN_PWM_BITS);
  ledcWrite(PIN_FAN_IN,  0);
  ledcWrite(PIN_FAN_OUT, 0);

  // Dimmer ZC
  pinMode(PIN_DIM, OUTPUT);
  digitalWrite(PIN_DIM, LOW);
  pinMode(PIN_ZC, INPUT_PULLUP);

  // Timer de 1 MHz (ticks en microsegundos) — API nueva:
  fireTimer = timerBegin(1000000);                // frecuencia base = 1 MHz
  timerAttachInterrupt(fireTimer, &onFireTimer);  // ISR de disparo

  // Interrupción cruce por cero
  attachInterrupt(digitalPinToInterrupt(PIN_ZC), isrZeroCross, RISING);

  // PID_v1
  myPID.SetOutputLimits(U_MIN, U_MAX);            // 0..100 %
  myPID.SetSampleTime(PID_SAMPLE_MS);             // 100 ms
  myPID.SetMode(AUTOMATIC);

  // Tareas
  xTaskCreatePinnedToCore(taskSense,     "sense",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskControl,   "control", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskActuators, "act",     4096, NULL, 1, NULL, 0);

  Serial.println("Inicio OK (core 3.x API).");
}

/* ===================== LOOP ===================== */
void loop() {
  static uint32_t t0 = millis();
  if (millis() - t0 > 1000) {
    t0 = millis();
    Serial.printf("T=%.2f°C  OK=%d  u=%.1f%%  SP=%.1f  safety=%d\n",
                  y_tempC, sensor_ok, u_duty, SETPOINT_C, safety_trip);
  }
  delay(10);
}
