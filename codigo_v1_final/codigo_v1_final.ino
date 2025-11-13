#include <SmoothThermistor.h>
#include <PID_v1.h>
#include <esp32-hal-timer.h>
#include <esp32-hal-ledc.h>
#include <driver/adc.h>

#include <FastAccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include "soc/gpio_struct.h"
#include <math.h>

/* ===================== HELPERS SALIDA (TODO POR Serial) ===================== */
static inline void OUT_print(const String &s){ Serial.print(s); }
static inline void OUT_print(const char* s) { Serial.print(s); }
static inline void OUT_println(const String &s){ Serial.println(s); }
static inline void OUT_println(const char* s) { Serial.println(s); }

/* =====================================================================
 *  BLOQUE 1: CONTROL DE TEMPERATURA + DIMMER + VENTILADORES (PIN 12 y 26)
 *  COMUNICACIÓN POR Serial @115200 (PID + mensajes)
 * ===================================================================== */

#define MAINS_FREQ_HZ        50
static const uint32_t HALF_CYCLE_US = (1000000UL / (MAINS_FREQ_HZ * 2));

double SETPOINT_C = 26.0;
const double SP_MIN = 0.0;
const double SP_MAX = 80.0;

// PID_v1 — Ts=0.1 s
double Kp = 2.3450;
double Ki = 1.0849;
double Kd = 0.9846;
const uint16_t PID_SAMPLE_MS = 100;

const double U_MIN = 25.0;   // 25 %
const double U_MAX = 100.0;  // 100 %

const int PIN_FAN_IN   = 12;
const int PIN_FAN_OUT  = 26;
const int FAN_PWM_HZ   = 25000;
const int FAN_PWM_BITS = 10;
const uint32_t FAN_MAX = (1u << FAN_PWM_BITS) - 1;

const double FAN_MIN_PCT = 0.0;
const double FAN_MAX_PCT = 100.0;

// Dimmer
const int PIN_ZC    = 27;
const int PIN_DIM   = 14;
const uint32_t GATE_PULSE_US = 75;
const uint32_t ZC_BLANK_US   = 500;

// Termistor NTC
const int PIN_NTC_ADC = 35;
SmoothThermistor ntc(
  PIN_NTC_ADC,
  ADC_SIZE_12_BIT,
  100000,
  100000,
  3950,
  25,
  25
);

const double TEMP_MIN_C = -10.0;
const double TEMP_MAX_C = 120.0;

volatile double y_tempC = 25.0;
volatile double u_duty  = 0.0;
PID myPID((double*)&y_tempC, (double*)&u_duty, &SETPOINT_C, Kp, Ki, Kd, DIRECT);

volatile bool safety_trip = false;
volatile bool sensor_ok   = true;

volatile double t_raw_last   = NAN;
volatile double fan_pct_last = 0.0;
volatile double fan_in_pct   = 0.0;
volatile double fan_out_pct  = 0.0;

hw_timer_t* fireTimer = nullptr;

volatile uint32_t u_delay_us = 0;
volatile bool gate_enable    = true;
volatile uint32_t last_zc_us = 0;

/* ===================== FILTROS TEMP ===================== */
#define USE_MEDIAN5       1
#define USE_EMA_IIR       1
#define USE_RATE_LIMITER  1

static const double SENSE_DT_S = 0.150;
static const double TEMP_TAU_S = 1.5;
static const double MAX_SLOPE_C_PER_S = 2.0;

static double ema_state = NAN;
static double last_filtered = NAN;
static double fan_cmd_smooth = NAN;

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

/* ===================== UTILS PID ===================== */
inline uint32_t dutyToDelayUs(double dutyPct) {
  double d = constrain(dutyPct, 0.0, 100.0);
  double alpha = (1.0 - d/100.0) * PI;
  uint32_t delayUs = (uint32_t)((alpha/PI) * HALF_CYCLE_US);
  if (delayUs < 200) delayUs = 200;
  if (delayUs > HALF_CYCLE_US - 200) delayUs = HALF_CYCLE_US - 200;
  return delayUs;
}

inline double heater_to_fan_inverse(double heaterPct) {
  double h = constrain(heaterPct, U_MIN, U_MAX);
  double norm = (h - U_MIN) / (U_MAX - U_MIN);
  double fan = FAN_MAX_PCT * (1.0 - norm);
  fan = constrain(fan, FAN_MIN_PCT, FAN_MAX_PCT);
  return fan;
}

inline double fan_smooth(double target) {
  const double alpha = 0.2;
  if (isnan(fan_cmd_smooth)) fan_cmd_smooth = target;
  fan_cmd_smooth = fan_cmd_smooth + alpha * (target - fan_cmd_smooth);
  return fan_cmd_smooth;
}

inline double clampSP(double sp){
  if (sp < SP_MIN) sp = SP_MIN;
  if (sp > SP_MAX) sp = SP_MAX;
  return sp;
}

/* ===================== BLOQUE 2 DEFINICIONES BÁSICAS (para usar en PID) ===================== */
// CNC 2 ejes

static const float PX[6] = { 20.0f, 200.0f, 200.0f,  20.0f,  20.0f, 200.0f };
static const float PY[6] = {  0.0f,   0.0f, 130.0f, 130.0f, 265.0f, 265.0f };

// Ejes
#define X_STEP_PIN     18
#define X_DIR_PIN      19
#define X_ENDSTOP_PIN  13
#define X_ENDSTOP_ACTIVE_LOW 1

#define Y_STEP_PIN     23 
#define Y_DIR_PIN      16
#define Y_ENDSTOP_PIN  32
#define Y_ENDSTOP_ACTIVE_LOW 1

// Ventilador PWM CNC
#define FAN_PWM_PIN    17

// Límites
const float X_MAX_MM = 220.0f;
const float Y_MAX_MM = 320.0f;

// Mecánica
float X_STEPS_PER_MM = 40.0f;
float Y_STEPS_PER_MM = 50.0f;
int   X_SPEED_HZ = 10000;
int   X_ACCEL    = 20000;
int   Y_SPEED_HZ = 10000;
int   Y_ACCEL    = 20000;

// Homing
const bool  X_HOME_TOWARD_NEG = true;
const bool  Y_HOME_TOWARD_NEG = true;
const float X_HOME_BACKOFF_MM = 3.0f;
const float Y_HOME_BACKOFF_MM = 3.0f;
const int   X_HOME_FAST_HZ = 5000;
const int   X_HOME_SLOW_HZ = 1200;
const int   Y_HOME_FAST_HZ = 5000;
const int   Y_HOME_SLOW_HZ = 1200;

// Direcciones
const bool X_DIR_INVERTED = true;
const bool Y_DIR_INVERTED = false;

FastAccelStepperEngine engine;
FastAccelStepper* stepperX = nullptr;
FastAccelStepper* stepperY = nullptr;

/* ========================= RTOS Infra CNC + PID ========================= */
#define EV_X_HOMED     (1<<0)
#define EV_Y_HOMED     (1<<1)
#define EV_MOTION_BUSY (1<<2)

static EventGroupHandle_t eg;

inline long  mmToStepsX(float mm){ return lroundf(mm * X_STEPS_PER_MM); }
inline long  mmToStepsY(float mm){ return lroundf(mm * Y_STEPS_PER_MM); }
inline float stepsToMmX(long st) { return (float)st / X_STEPS_PER_MM; }
inline float stepsToMmY(long st) { return (float)st / Y_STEPS_PER_MM; }
inline float getXmm() { return stepsToMmX(stepperX->getCurrentPosition()); }
inline float getYmm() { return stepsToMmY(stepperY->getCurrentPosition()); }

inline bool endstopXPressed(){ int v=digitalRead(X_ENDSTOP_PIN); return X_ENDSTOP_ACTIVE_LOW ? (v==LOW):(v==HIGH); }
inline bool endstopYPressed(){ int v=digitalRead(Y_ENDSTOP_PIN); return Y_ENDSTOP_ACTIVE_LOW ? (v==LOW):(v==HIGH); }

/* ========= SOLO control PID cuando la CNC está en HOME (0,0) ========= */
bool isAtHomePos() {
  if (stepperX == nullptr || stepperY == nullptr || eg == nullptr) return false;

  EventBits_t b = xEventGroupGetBits(eg);
  // Debe estar homed en ambos ejes
  if ((b & EV_X_HOMED) == 0 || (b & EV_Y_HOMED) == 0) return false;

  float x = getXmm();
  float y = getYmm();

  const float TOL = 0.5f; // tolerancia en mm
  bool atHome = (x > -TOL && x < TOL && y > -TOL && y < TOL);
  return atHome;
}

/* ===================== ISRs DIMMER ===================== */
void IRAM_ATTR onFireTimer() {
  static uint32_t last_fire = 0;
  uint32_t now = micros();
  if (now - last_fire < 2000) {
    timerStop(fireTimer);
    return;
  }
  last_fire = now;

  if (!gate_enable) {
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
  if (now - last_zc_us < ZC_BLANK_US) return;
  last_zc_us = now;

  if (!gate_enable) return;

  timerStop(fireTimer);
  timerWrite(fireTimer, 0);
  timerAlarm(fireTimer, u_delay_us, false, 0);
  timerStart(fireTimer);
}

/* ===================== TAREAS PID ===================== */

/* Histeresis para sensor_ok */
static uint8_t bad_count  = 0;
static uint8_t good_count = 0;
const uint8_t BAD_THRESHOLD  = 5;  // lecturas malas seguidas para marcar sensor_ok = false
const uint8_t GOOD_THRESHOLD = 3;  // lecturas buenas seguidas para volver a sensor_ok = true

void taskSense(void* pv) {
  for (;;) {
    double t_raw = ntc.temperature();
    t_raw_last = t_raw;

    // Lectura válida si no es NaN y está dentro de rango
    bool ok_now = !isnan(t_raw) && t_raw > TEMP_MIN_C && t_raw < TEMP_MAX_C;

    if (!ok_now) {
      if (bad_count < 255) bad_count++;
      good_count = 0;
    } else {
      if (good_count < 255) good_count++;
      bad_count = 0;
    }

    // Actualizamos sensor_ok con histéresis
    if (bad_count >= BAD_THRESHOLD) {
      sensor_ok = false;
    } else if (good_count >= GOOD_THRESHOLD) {
      sensor_ok = true;
    }

    // Solo filtramos y actualizamos y_tempC si el sensor está OK
    if (sensor_ok && ok_now) {
      double t_filt = filter_temperature(t_raw);
      y_tempC = t_filt;
    }

    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

void taskControl(void* pv) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(PID_SAMPLE_MS);
  for (;;) {
    vTaskDelayUntil(&last, period);

    bool atHome = isAtHomePos();

    // REGLA: el PID SOLO funciona cuando la CNC está en HOME (0,0)
    if (safety_trip || !sensor_ok || !atHome) {
      gate_enable = false;
      if (myPID.GetMode() == AUTOMATIC)
        myPID.SetMode(MANUAL);
      u_duty = 0.0;
      u_delay_us = HALF_CYCLE_US;
      continue;
    }

    // Aquí: estamos en home, sensor ok y sin safety_trip -> PID activo
    if (myPID.GetMode() == MANUAL) {
      myPID.SetMode(AUTOMATIC);
    }
    gate_enable = true;

    myPID.Compute();
    u_delay_us = dutyToDelayUs(u_duty);
  }
}

void taskActuators(void* pv) {
  for (;;) {
    double fanPct;

    if (safety_trip || !sensor_ok) {
      fanPct = FAN_MAX_PCT;
    } else {
      double heaterPct = u_duty;
      double fanTarget = heater_to_fan_inverse(heaterPct);
      fanPct = fan_smooth(fanTarget);
    }

    uint32_t duty_counts = (uint32_t)(fanPct * FAN_MAX / 100.0);
    ledcWrite(PIN_FAN_IN,  duty_counts);
    ledcWrite(PIN_FAN_OUT, duty_counts);

    fan_in_pct  = fanPct;
    fan_out_pct = fanPct;
    fan_pct_last = fanPct;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* ========================= FAN CONTROL CNC ========================= */
static volatile uint8_t  fan_pwm     = 0;
static volatile bool     fan_on      = false;
static volatile uint16_t fan_pwm_cnt = 0;
static hw_timer_t*       fanTimer    = nullptr;

void IRAM_ATTR fanTimerISR() {
  uint8_t duty = fan_on ? fan_pwm : 0;
  if (fan_pwm_cnt < duty) GPIO.out_w1ts = (1UL << FAN_PWM_PIN);
  else                    GPIO.out_w1tc = (1UL << FAN_PWM_PIN);
  fan_pwm_cnt++;
  if (fan_pwm_cnt >= 256) fan_pwm_cnt = 0;
}

void fanOn()    { fan_on = true;  if (fan_pwm == 0) fan_pwm = 255; }
void fanOff()   { fan_on = false; }
void fanSetPower(uint8_t p) {
  fan_pwm = p;
  fan_on  = (p > 0);
}

/* ========================= LIMITES Y HOMING ========================= */
static bool checkLimitX(float mmTarget){
  EventBits_t b = xEventGroupGetBits(eg);
  if (!(b & EV_X_HOMED)) return true;
  return (mmTarget >= 0.0f && mmTarget <= X_MAX_MM);
}
static bool checkLimitY(float mmTarget){
  EventBits_t b = xEventGroupGetBits(eg);
  if (!(b & EV_Y_HOMED)) return true;
  return (mmTarget >= 0.0f && mmTarget <= Y_MAX_MM);
}

static void waitStepper(FastAccelStepper* s){
  while (s->isRunning()) vTaskDelay(pdMS_TO_TICKS(1));
}

static bool doHomeAxis(FastAccelStepper* s,
                       bool (*endstopPressed)(),
                       bool homeTowardNeg,
                       float backoff_mm,
                       int fast_hz,
                       int slow_hz,
                       int speed_normal_hz,
                       int accel_normal,
                       float steps_per_mm)
{
  OUT_println("  - Búsqueda rápida...");
  s->setAcceleration(accel_normal);
  s->setSpeedInHz(fast_hz);

  long direction = homeTowardNeg ? -1 : 1;
  long maxSteps  = lroundf(1000.0f * steps_per_mm);
  s->move(direction * maxSteps);

  while (s->isRunning()){
    if (endstopPressed()){ s->forceStopAndNewPosition(0); break; }
    vTaskDelay(1);
  }

  if (!endstopPressed()){
    OUT_println("  ! Endstop no detectado");
    s->setSpeedInHz(speed_normal_hz);
    s->setAcceleration(accel_normal);
    return false;
  }

  OUT_println("  - Retroceso...");
  s->setSpeedInHz(slow_hz);
  s->move(-direction * lroundf(backoff_mm * steps_per_mm));
  waitStepper(s);
  vTaskDelay(pdMS_TO_TICKS(50));

  OUT_println("  - Aproximación lenta...");
  s->move(direction * lroundf(backoff_mm * 2.0f * steps_per_mm));
  while (s->isRunning()){
    if (endstopPressed()){ s->forceStopAndNewPosition(0); break; }
    vTaskDelay(1);
  }

  s->setCurrentPosition(0);
  s->setSpeedInHz(speed_normal_hz);
  s->setAcceleration(accel_normal);
  OUT_println("  OK: pos=0");
  return true;
}

static void preUnlatch(FastAccelStepper* s, bool homeTowardNeg, float steps_per_mm, float travel_mm){
  int  away_dir = homeTowardNeg ? +1 : -1;
  long st       = lroundf(travel_mm * steps_per_mm) * away_dir;
  if (st == 0) return;
  s->move(st);
  waitStepper(s);
  vTaskDelay(pdMS_TO_TICKS(30));
}

/* ========================= MOVIMIENTOS ========================= */
static bool moveToXYAbsSync(float x_mm, float y_mm){
  if (!checkLimitX(x_mm) || !checkLimitY(y_mm)){ OUT_println("  ! Limite excedido"); return false; }
  float curx = getXmm(), cury = getYmm();
  long dx = mmToStepsX(x_mm - curx);
  long dy = mmToStepsY(y_mm - cury);
  long adx = labs(dx), ady = labs(dy);
  if (adx==0 && ady==0) return true;

  int  baseSpeed = min(X_SPEED_HZ, Y_SPEED_HZ);
  int  baseAccel = min(X_ACCEL,    Y_ACCEL);
  long maxSteps  = max(adx, ady);
  auto scaled = [&](long a, int base)->int{
    if (a==0) return 0;
    long v = lround((double)base * (double)a / (double)maxSteps);
    if (v < 100) v = 100;
    return (int)v;
  };
  int vx = scaled(adx, baseSpeed), vy = scaled(ady, baseSpeed);
  int ax = scaled(adx, baseAccel), ay = scaled(ady, baseAccel);

  int old_vx = X_SPEED_HZ, old_ax = X_ACCEL;
  int old_vy = Y_SPEED_HZ, old_ay = Y_ACCEL;

  if (adx>0){ stepperX->setSpeedInHz(vx); stepperX->setAcceleration(ax); }
  if (ady>0){ stepperY->setSpeedInHz(vy); stepperY->setAcceleration(ay); }

  if (adx>0) stepperX->move(dx);
  if (ady>0) stepperY->move(dy);

  while ((adx>0 && stepperX->isRunning()) || (ady>0 && stepperY->isRunning())){
    vTaskDelay(1);
  }

  stepperX->setSpeedInHz(old_vx); stepperX->setAcceleration(old_ax);
  stepperY->setSpeedInHz(old_vy); stepperY->setAcceleration(old_ay);
  return true;
}

static bool moveRelX(float mm){
  float dest = getXmm() + mm;
  if (!checkLimitX(dest)) return false;
  stepperX->move(mmToStepsX(mm));
  waitStepper(stepperX);
  return true;
}
static bool moveRelY(float mm){
  float dest = getYmm() + mm;
  if (!checkLimitY(dest)) return false;
  stepperY->move(mmToStepsY(mm));
  waitStepper(stepperY);
  return true;
}

/* ========================= TAREAS INTERFAZ & MOVIMIENTO ========================= */
typedef enum : uint8_t {
  CMD_HELP,
  CMD_ANALISIS,
  CMD_HOME_X, CMD_HOME_Y, CMD_HOME_BOTH,
  CMD_G0X, CMD_G0Y, CMD_G0XY,
  CMD_P1, CMD_P2, CMD_P3, CMD_P4, CMD_P5, CMD_P6,
  CMD_REL_MMX, CMD_REL_MMY,
  CMD_POSQ,
  CMD_FAN_ON, CMD_FAN_OFF, CMD_FAN_Q,
  CMD_FAN_POWER,
  // PID
  CMD_PID_KP,
  CMD_PID_KI,
  CMD_PID_KD,
  CMD_PID_SP_ABS,
  CMD_PID_SP_INC,
  CMD_PID_SP_DEC,
  CMD_PID_SHOW,
  CMD_PID_SENS,
  CMD_PID_SAFE_ON,
  CMD_PID_SAFE_OFF
} CmdType;

typedef struct {
  CmdType type;
  float   value;
} Command;

static QueueHandle_t qCmd;
static TaskHandle_t hUI   = nullptr;
static TaskHandle_t hUtil = nullptr;
static TaskHandle_t hMotion = nullptr;

static String cmdBuf;

static void setBusy(bool on){
  if (on) xEventGroupSetBits(eg, EV_MOTION_BUSY);
  else    xEventGroupClearBits(eg, EV_MOTION_BUSY);
}

static void uiSend(CmdType t, float v=0){
  Command c{t, v};
  xQueueSend(qCmd, &c, portMAX_DELAY);
}

/* --- Parser ÚNICO (PID + MOTORES) --- */
static void parseLineAndDispatch(const String& sIn){
  String s = sIn; s.trim(); s.toLowerCase();
  if (!s.length()) return;

  // AYUDA GENERAL
  if (s=="help" || s=="?") {
    OUT_println("\nComandos PID:");
    OUT_println("  kp <v>, ki <v>, kd <v>");
    OUT_println("  sp <v>, sp+ <d>, sp- <d>");
    OUT_println("  show, sens, safe on, safe off");
    OUT_println("\nComandos MOTORES:");
    OUT_println("  analisis");
    OUT_println("  p1..p6");
    OUT_println("  home | homex | homey");
    OUT_println("  g0   | g0x   | g0y");
    OUT_println("  mmx=<n> | mmy=<n>");
    OUT_println("  pos?");
    OUT_println("  fanon | fanoff | fan? | fanp=<0–255>\n");
    return;
  }

  // === PID ===
  if      (s.startsWith("kp ")) uiSend(CMD_PID_KP, s.substring(3).toFloat());
  else if (s.startsWith("ki ")) uiSend(CMD_PID_KI, s.substring(3).toFloat());
  else if (s.startsWith("kd ")) uiSend(CMD_PID_KD, s.substring(3).toFloat());
  else if (s.startsWith("sp+ ")) uiSend(CMD_PID_SP_INC, s.substring(4).toFloat());
  else if (s.startsWith("sp- ")) uiSend(CMD_PID_SP_DEC, s.substring(4).toFloat());
  else if (s.startsWith("sp "))  uiSend(CMD_PID_SP_ABS, s.substring(3).toFloat());
  else if (s=="show")    uiSend(CMD_PID_SHOW);
  else if (s=="sens")    uiSend(CMD_PID_SENS);
  else if (s=="safe on") uiSend(CMD_PID_SAFE_ON);
  else if (s=="safe off")uiSend(CMD_PID_SAFE_OFF);

  // === MOTORES ===
  else if (s=="analisis")  uiSend(CMD_ANALISIS);
  else if (s=="homex")     uiSend(CMD_HOME_X);
  else if (s=="homey")     uiSend(CMD_HOME_Y);
  else if (s=="home")      uiSend(CMD_HOME_BOTH);
  else if (s=="g0x")       uiSend(CMD_G0X);
  else if (s=="g0y")       uiSend(CMD_G0Y);
  else if (s=="g0")        uiSend(CMD_G0XY);
  else if (s=="p1")        uiSend(CMD_P1);
  else if (s=="p2")        uiSend(CMD_P2);
  else if (s=="p3")        uiSend(CMD_P3);
  else if (s=="p4")        uiSend(CMD_P4);
  else if (s=="p5")        uiSend(CMD_P5);
  else if (s=="p6")        uiSend(CMD_P6);
  else if (s=="pos?")      uiSend(CMD_POSQ);
  else if (s=="fanon")     uiSend(CMD_FAN_ON);
  else if (s=="fanoff")    uiSend(CMD_FAN_OFF);
  else if (s=="fan?")      uiSend(CMD_FAN_Q);
  else if (s.startsWith("fanp=")) uiSend(CMD_FAN_POWER, s.substring(5).toFloat());
  else if (s.startsWith("mmx="))  uiSend(CMD_REL_MMX,   s.substring(4).toFloat());
  else if (s.startsWith("mmy="))  uiSend(CMD_REL_MMY,   s.substring(4).toFloat());
  else OUT_println("Comando invalido. Escribe 'help'.");
}

static void handleStream(Stream& in, String& buf) {
  while (in.available()){
    char c = in.read();
    if (c=='\r' || c=='\n'){
      String line = buf; buf = "";
      parseLineAndDispatch(line);
    } else {
      buf += c;
    }
  }
}

static void vTaskUI(void*){
  OUT_println("ESP32 listo. Usa 'help' para ver comandos (PID + MOTORES).");
  for(;;){
    handleStream(Serial, cmdBuf);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void vTaskUtil(void*){
  TickType_t last = xTaskGetTickCount();
  for(;;){
    vTaskDelayUntil(&last, pdMS_TO_TICKS(250));
  }
}

/* --- Motion / Planner --- */
static void homeX(){
  setBusy(true);
  OUT_println("\n=== HOMING X ===");
  preUnlatch(stepperX, X_HOME_TOWARD_NEG, X_STEPS_PER_MM, 5.0f);
  bool ok = doHomeAxis(stepperX, endstopXPressed, X_HOME_TOWARD_NEG,
                       X_HOME_BACKOFF_MM, X_HOME_FAST_HZ, X_HOME_SLOW_HZ,
                       X_SPEED_HZ, X_ACCEL, X_STEPS_PER_MM);
  if (ok) xEventGroupSetBits(eg, EV_X_HOMED); else xEventGroupClearBits(eg, EV_X_HOMED);
  setBusy(false);
}
static void homeY(){
  setBusy(true);
  OUT_println("\n=== HOMING Y ===");
  preUnlatch(stepperY, Y_HOME_TOWARD_NEG, Y_STEPS_PER_MM, 5.0f);
  bool ok = doHomeAxis(stepperY, endstopYPressed, Y_HOME_TOWARD_NEG,
                       Y_HOME_BACKOFF_MM, Y_HOME_FAST_HZ, Y_HOME_SLOW_HZ,
                       Y_SPEED_HZ, Y_ACCEL, Y_STEPS_PER_MM);
  if (ok) xEventGroupSetBits(eg, EV_Y_HOMED); else xEventGroupClearBits(eg, EV_Y_HOMED);
  setBusy(false);
}

static void go0X(){ setBusy(true); (void)moveToXYAbsSync(0.0f, getYmm()); setBusy(false); OUT_println("OK X=0"); }
static void go0Y(){ setBusy(true); (void)moveToXYAbsSync(getXmm(), 0.0f); setBusy(false); OUT_println("OK Y=0"); }

static void doAnalisis(){
  setBusy(true);
  homeX(); homeY();
  EventBits_t b = xEventGroupGetBits(eg);
  if (!(b & EV_X_HOMED) || !(b & EV_Y_HOMED)){
    OUT_println("! Homing falló. Aborto.");
    setBusy(false);
    return;
  }
  for (uint8_t i=0;i<6;i++){
    char msg[64];
    snprintf(msg, sizeof(msg), "→ P%d = (%.1f, %.1f)", i+1, PX[i], PY[i]);
    OUT_println(msg);
    if (!moveToXYAbsSync(PX[i], PY[i])){ OUT_println("  ! Límite; Analisis abortado"); break; }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  OUT_println("ANALISIS listo.");
  setBusy(false);
}

static void gotoPoint(uint8_t idx){
  if (idx<1 || idx>6) return;
  EventBits_t b = xEventGroupGetBits(eg);
  if (!(b & EV_X_HOMED) || !(b & EV_Y_HOMED)) { homeX(); homeY(); }
  char msg[32]; snprintf(msg, sizeof(msg), "→ Ir a P%u", idx); OUT_println(msg);
  setBusy(true);
  (void)moveToXYAbsSync(PX[idx-1], PY[idx-1]);
  setBusy(false);
  OUT_println("OK punto.");
}

static void vTaskMotion(void*){
  Command c;
  for(;;){
    if (xQueueReceive(qCmd, &c, portMAX_DELAY) == pdTRUE){
      switch(c.type){
        /* === Comandos MOTORES === */
        case CMD_HELP:
          OUT_println("\nComandos PID:");
          OUT_println("  kp <v>, ki <v>, kd <v>");
          OUT_println("  sp <v>, sp+ <d>, sp- <d>");
          OUT_println("  show, sens, safe on, safe off");
          OUT_println("\nComandos MOTORES:");
          OUT_println("  analisis");
          OUT_println("  p1..p6");
          OUT_println("  home | homex | homey");
          OUT_println("  g0   | g0x   | g0y");
          OUT_println("  mmx=<n> | mmy=<n>");
          OUT_println("  pos?");
          OUT_println("  fanon | fanoff | fan? | fanp=<0–255>\n");
          break;

        case CMD_FAN_ON:     fanOn();  OUT_println("Fan PWM (pin 17) ON"); break;
        case CMD_FAN_OFF:    fanOff(); OUT_println("Fan PWM (pin 17) OFF"); break;
        case CMD_FAN_Q: {
          char msg[32];
          snprintf(msg,sizeof(msg),"Fan17: %s PWM=%u", fan_on?"ON":"OFF", fan_pwm);
          OUT_println(msg);
        } break;
        case CMD_FAN_POWER: {
          uint8_t p = (uint8_t)c.value;
          fanSetPower(p);
          char msg[32]; snprintf(msg,sizeof(msg),"Fan17 PWM = %u", p);
          OUT_println(msg);
        } break;

        case CMD_HOME_X:     homeX(); break;
        case CMD_HOME_Y:     homeY(); break;
        case CMD_HOME_BOTH:  homeX(); homeY(); break;

        case CMD_G0X:        go0X(); break;
        case CMD_G0Y:        go0Y(); break;
        case CMD_G0XY:       setBusy(true); (void)moveToXYAbsSync(0,0); setBusy(false); OUT_println("OK X=0,Y=0"); break;

        case CMD_P1: case CMD_P2: case CMD_P3:
        case CMD_P4: case CMD_P5: case CMD_P6:
          gotoPoint((uint8_t)(c.type - CMD_P1 + 1)); break;

        case CMD_REL_MMX:
          setBusy(true); if (!moveRelX(c.value)) OUT_println("! Limite X"); setBusy(false); break;
        case CMD_REL_MMY:
          setBusy(true); if (!moveRelY(c.value)) OUT_println("! Limite Y"); setBusy(false); break;

        case CMD_POSQ: {
          char msg[80];
          snprintf(msg, sizeof(msg), "X: %.2f mm | Y: %.2f mm", getXmm(), getYmm());
          OUT_println(msg);
        } break;

        case CMD_ANALISIS:   doAnalisis(); break;

        /* === Comandos PID === */
        case CMD_PID_KP:
          Kp = c.value;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print("Nuevo Kp = "); Serial.println(Kp, 4);
          break;
        case CMD_PID_KI:
          Ki = c.value;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print("Nuevo Ki = "); Serial.println(Ki, 4);
          break;
        case CMD_PID_KD:
          Kd = c.value;
          myPID.SetTunings(Kp, Ki, Kd);
          Serial.print("Nuevo Kd = "); Serial.println(Kd, 4);
          break;
        case CMD_PID_SP_ABS:
          SETPOINT_C = clampSP(c.value);
          Serial.print("Nuevo SP = "); Serial.print(SETPOINT_C, 2); Serial.println(" °C");
          break;
        case CMD_PID_SP_INC:
          SETPOINT_C = clampSP(SETPOINT_C + c.value);
          Serial.print("SP aumentado: "); Serial.print(SETPOINT_C, 2); Serial.println(" °C");
          break;
        case CMD_PID_SP_DEC:
          SETPOINT_C = clampSP(SETPOINT_C - c.value);
          Serial.print("SP reducido: "); Serial.print(SETPOINT_C, 2); Serial.println(" °C");
          break;
        case CMD_PID_SHOW:
          Serial.printf("PID: Kp=%.4f  Ki=%.4f  Kd=%.4f | SP=%.2f °C | T=%.2f °C | U=%.2f%% | gate=%d\n",
                        Kp, Ki, Kd, SETPOINT_C, y_tempC, u_duty, (int)gate_enable);
          break;
        case CMD_PID_SENS:
          Serial.printf("T_raw=%.2f  T_filt=%.2f  ok=%d\n", t_raw_last, y_tempC, sensor_ok ? 1 : 0);
          break;
        case CMD_PID_SAFE_ON:
          safety_trip = true;
          Serial.println("Safety TRIP activado.");
          break;
        case CMD_PID_SAFE_OFF:
          safety_trip = false;
          Serial.println("Safety TRIP liberado.");
          break;
      }
    }
  }
}

/* ========================= SETUP ========================= */
void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC NTC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // PWM FANS (12 y 26)
  ledcAttach(PIN_FAN_IN,  FAN_PWM_HZ, FAN_PWM_BITS);
  ledcAttach(PIN_FAN_OUT, FAN_PWM_HZ, FAN_PWM_BITS);
  ledcWrite(PIN_FAN_IN,  0);
  ledcWrite(PIN_FAN_OUT, 0);

  // Dimmer / ZC
  pinMode(PIN_DIM, OUTPUT);
  digitalWrite(PIN_DIM, LOW);
  pinMode(PIN_ZC, INPUT_PULLUP);

  fireTimer = timerBegin(1000000); // 1 MHz
  timerAttachInterrupt(fireTimer, &onFireTimer);
  attachInterrupt(digitalPinToInterrupt(PIN_ZC), isrZeroCross, CHANGE);

  myPID.SetOutputLimits(U_MIN, U_MAX);
  myPID.SetSampleTime(PID_SAMPLE_MS);
  myPID.SetMode(AUTOMATIC);

  // Endstops
  pinMode(X_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_ENDSTOP_PIN, INPUT_PULLUP);

  // Ventilador CNC PWM en pin 17
  pinMode(FAN_PWM_PIN, OUTPUT);
  GPIO.out_w1tc = (1UL << FAN_PWM_PIN);
  fanTimer = timerBegin(250000);   // 250 kHz -> ~976 Hz (256 niveles)
  timerAttachInterrupt(fanTimer, fanTimerISR);
  timerAlarm(fanTimer, 1, true, 0);
  timerStart(fanTimer);
  fanSetPower(0);

  // Steppers
  engine.init();

  stepperX = engine.stepperConnectToPin(X_STEP_PIN);
  if (!stepperX){ OUT_println("Error X"); while(1){} }
  stepperX->setDirectionPin(X_DIR_PIN, X_DIR_INVERTED);
  stepperX->setAutoEnable(false);
  stepperX->setSpeedInHz(X_SPEED_HZ);
  stepperX->setAcceleration(X_ACCEL);

  stepperY = engine.stepperConnectToPin(Y_STEP_PIN);
  if (!stepperY){ OUT_println("Error Y"); while(1){} }
  stepperY->setDirectionPin(Y_DIR_PIN, Y_DIR_INVERTED);
  stepperY->setAutoEnable(false);
  stepperY->setSpeedInHz(Y_SPEED_HZ);
  stepperY->setAcceleration(Y_ACCEL);

  // === RTOS: primero el EventGroup, luego homing inicial, luego la cola y tareas ===
  eg   = xEventGroupCreate();

  OUT_println("\n=== HOMING INICIAL AL ENCENDER ===");
  homeX();
  homeY();
  OUT_println("Homing inicial completo. Posicionando en (0,0).");
  // Por seguridad, mandamos a (0,0) absoluto (por si quedó algún offset mínimo)
  (void)moveToXYAbsSync(0.0f, 0.0f);

  // Cola de comandos y tareas
  qCmd = xQueueCreate(16, sizeof(Command));

  xTaskCreatePinnedToCore(taskSense,     "sense",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskControl,   "control", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskActuators, "act",     4096, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore(vTaskUI,       "UI",     4096, nullptr, 2, &hUI,     0);
  xTaskCreatePinnedToCore(vTaskUtil,     "UTIL",   2048, nullptr, 1, &hUtil,   0);
  xTaskCreatePinnedToCore(vTaskMotion,   "MOTION", 4096, nullptr, 3, &hMotion, 1);

  OUT_println("Sistema listo. Usa 'help' en el Monitor Serie (115200).");
}

/* ===================== LOOP ===================== */
void loop() {
  // Mensaje de estado cada 1 s
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    bool atHome = isAtHomePos();
    Serial.printf("T=%.2f C SP=%.2f \n",
                  y_tempC,  SETPOINT_C);
  }

  vTaskDelay(pdMS_TO_TICKS(50)); // Todo corre en tareas
}
