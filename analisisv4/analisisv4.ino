/*
  ESP32 + FastAccelStepper (2 ejes) + Ventilador PWM por timer (1 pin)
  + Bluetooth clásico SPP (Serial Bluetooth)
  - Multitarea y dual-core (FreeRTOS)
  - Ventilador: PWM ~1 kHz generado por HW timer (SIN LEDC, API 3.x)
  - Comandos:
      analisis
      p1..p6
      home | homex | homey
      g0   | g0x   | g0y
      mmx=<n> | mmy=<n>
      pos?
      fanon | fanoff | fan? | fanp=<0–255>
*/
#include <FastAccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include "soc/gpio_struct.h"

// ===== Bluetooth clásico (SPP) =====
#include <BluetoothSerial.h>
BluetoothSerial BT;
static bool btReady = false;

// ===== Helpers salida a USB + BT =====
static inline void OUT_print(const String &s){ Serial.print(s); if (btReady && BT.hasClient()) BT.print(s); }
static inline void OUT_print(const char* s) { Serial.print(s); if (btReady && BT.hasClient()) BT.print(s); }
static inline void OUT_println(const String &s){ OUT_print(s + "\n"); }
static inline void OUT_println(const char* s) { OUT_print(String(s) + "\n"); }

// ========================= CONFIGURACIÓN HW =========================
/* Puntos P1..P6 */
static const float PX[6] = { 20.0f, 200.0f, 200.0f,  20.0f,  20.0f, 200.0f };
static const float PY[6] = {  0.0f,   0.0f, 130.0f, 130.0f, 265.0f, 265.0f };

// Eje X
#define X_STEP_PIN     18
#define X_DIR_PIN      19
#define X_ENDSTOP_PIN  13
#define X_ENDSTOP_ACTIVE_LOW 1
// Eje Y
#define Y_STEP_PIN     23 
#define Y_DIR_PIN      16
#define Y_ENDSTOP_PIN  32   //Antes 27
#define Y_ENDSTOP_ACTIVE_LOW 1

// ---------- Ventilador PWM por timer (un solo pin) ----------
#define FAN_PWM_PIN    17     // EN del L298N / pin PWM único

// Límites
const float X_MAX_MM = 220.0f;
const float Y_MAX_MM = 320.0f;

// Mecánica / Dinámica
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

// ========================= MOTOR ENGINE =========================
FastAccelStepperEngine engine;
FastAccelStepper* stepperX = nullptr;
FastAccelStepper* stepperY = nullptr;

// ========================= HELPERS =========================
inline long  mmToStepsX(float mm){ return lroundf(mm * X_STEPS_PER_MM); }
inline long  mmToStepsY(float mm){ return lroundf(mm * Y_STEPS_PER_MM); }
inline float stepsToMmX(long st) { return (float)st / X_STEPS_PER_MM; }
inline float stepsToMmY(long st) { return (float)st / Y_STEPS_PER_MM; }
inline float getXmm() { return stepsToMmX(stepperX->getCurrentPosition()); }
inline float getYmm() { return stepsToMmY(stepperY->getCurrentPosition()); }

inline bool endstopXPressed(){ int v=digitalRead(X_ENDSTOP_PIN); return X_ENDSTOP_ACTIVE_LOW ? (v==LOW):(v==HIGH); }
inline bool endstopYPressed(){ int v=digitalRead(Y_ENDSTOP_PIN); return Y_ENDSTOP_ACTIVE_LOW ? (v==LOW):(v==HIGH); }

// ========================= RTOS Infra =========================
#define EV_X_HOMED     (1<<0)
#define EV_Y_HOMED     (1<<1)
#define EV_MOTION_BUSY (1<<2)

static EventGroupHandle_t eg;

typedef enum : uint8_t {
  CMD_HELP,
  CMD_ANALISIS,
  CMD_HOME_X, CMD_HOME_Y, CMD_HOME_BOTH,
  CMD_G0X, CMD_G0Y, CMD_G0XY,
  CMD_P1, CMD_P2, CMD_P3, CMD_P4, CMD_P5, CMD_P6,
  CMD_REL_MMX, CMD_REL_MMY,
  CMD_POSQ,
  CMD_FAN_ON, CMD_FAN_OFF, CMD_FAN_Q,
  CMD_FAN_POWER
} CmdType;

typedef struct {
  CmdType type;
  float   value;
} Command;

static QueueHandle_t qCmd;

// ========================= FAN CONTROL (PWM por timer API 3.x) =========================
static volatile uint8_t  fan_pwm     = 0;     // 0..255
static volatile bool     fan_on      = false; // ON/OFF lógico
static volatile uint16_t fan_pwm_cnt = 0;     // contador 0..255
static hw_timer_t*       fanTimer    = nullptr;

void IRAM_ATTR fanTimerISR() {
  uint8_t duty = fan_on ? fan_pwm : 0;
  if (fan_pwm_cnt < duty) GPIO.out_w1ts = (1UL << FAN_PWM_PIN);  // HIGH rápido
  else                    GPIO.out_w1tc = (1UL << FAN_PWM_PIN);  // LOW rápido
  fan_pwm_cnt++;
  if (fan_pwm_cnt >= 256) fan_pwm_cnt = 0;
}

// APIs de control
void fanApply() { /* el ISR lee fan_on/fan_pwm continuamente */ }
void fanOn()    { fan_on = true;  if (fan_pwm == 0) fan_pwm = 255; }
void fanOff()   { fan_on = false; }
void fanSetPower(uint8_t p) {
  fan_pwm = p;
  fan_on  = (p > 0);
}

// ========================= LIMITES =========================
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

// ========================= HOMING =========================
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

// ========================= MOVIMIENTOS =========================
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

// ========================= TAREAS =========================
/* --- TASK: User Interface (Core 0) --- */
static TaskHandle_t hUI = nullptr;
static String cmdBufUSB, cmdBufBT;

static void uiSend(CmdType t, float v=0){
  Command c{t, v};
  xQueueSend(qCmd, &c, portMAX_DELAY);
}

static void parseLineAndDispatch(const String& sIn){
  String s = sIn; s.trim(); s.toLowerCase();
  if (!s.length()) return;
  if (s=="help" || s=="?") uiSend(CMD_HELP);
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
  else OUT_println("Comando desconocido. 'help'");
}

static void handleInputStream(Stream& in, String& buf) {
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
  OUT_println("\nComandos:");
  OUT_println("  analisis");
  OUT_println("  p1..p6");
  OUT_println("  home | homex | homey");
  OUT_println("  g0   | g0x   | g0y");
  OUT_println("  mmx=<n> | mmy=<n>");
  OUT_println("  pos?");
  OUT_println("  fanon | fanoff | fan? | fanp=<0-255>\n");

  for(;;){
    // entrada por USB
    handleInputStream(Serial, cmdBufUSB);
    // entrada por Bluetooth clásico
    if (btReady && BT.hasClient()) handleInputStream(BT, cmdBufBT);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* --- TASK: Utilities (Core 0) --- */
static TaskHandle_t hUtil = nullptr;
static void vTaskUtil(void*){
  TickType_t last = xTaskGetTickCount();
  for(;;){
    // sitio para watchdog/telemetría
    vTaskDelayUntil(&last, pdMS_TO_TICKS(250));
  }
}

/* --- TASK: Motion / Planner (Core 1) --- */
static TaskHandle_t hMotion = nullptr;

static void setBusy(bool on){
  if (on) xEventGroupSetBits(eg, EV_MOTION_BUSY);
  else    xEventGroupClearBits(eg, EV_MOTION_BUSY);
}

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
        case CMD_HELP:
          OUT_println("\nComandos:");
          OUT_println("  analisis");
          OUT_println("  p1..p6");
          OUT_println("  home | homex | homey");
          OUT_println("  g0   | g0x   | g0y");
          OUT_println("  mmx=<n> | mmy=<n>");
          OUT_println("  pos?");
          OUT_println("  fanon | fanoff | fan? | fanp=<0–255>\n");
          break;

        case CMD_FAN_ON:     fanOn();  OUT_println("Fan ON (100%)"); break;
        case CMD_FAN_OFF:    fanOff(); OUT_println("Fan OFF"); break;
        case CMD_FAN_Q:    { char msg[32]; snprintf(msg,sizeof(msg),"Fan: %s PWM=%u", fan_on?"ON":"OFF", fan_pwm); OUT_println(msg);} break;
        case CMD_FAN_POWER: {
          uint8_t p = (uint8_t)c.value;
          fanSetPower(p);
          char msg[32]; snprintf(msg,sizeof(msg),"Fan PWM = %u", p); OUT_println(msg);
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
      }
    }
  }
}

// ========================= SETUP/LOOP =========================
void setup(){
  Serial.begin(115200);
  delay(200);

  // ===== Bluetooth SPP =====
  // Si quieres PIN: descomenta la siguiente línea ANTES de begin()
  // BT.setPin("1234");
  btReady = BT.begin("CNC-ESP32");  // nombre del dispositivo
  if (btReady) OUT_println("[BT] Bluetooth SPP listo. Empareja como 'CNC-ESP32'.");
  else         OUT_println("[BT] Error iniciando Bluetooth.");

  pinMode(X_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_ENDSTOP_PIN, INPUT_PULLUP);

  // --- Ventilador PWM por timer (API 3.x) ---
  pinMode(FAN_PWM_PIN, OUTPUT);
  GPIO.out_w1tc = (1UL << FAN_PWM_PIN);      // LOW al inicio
  fanTimer = timerBegin(250000);             // 250 kHz -> ~976 Hz PWM (256 niveles)
  timerAttachInterrupt(fanTimer, fanTimerISR);
  timerAlarm(fanTimer, 1, true, 0);
  timerStart(fanTimer);
  fanSetPower(0);

  // --- Steppers ---
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

  // RTOS infra
  eg   = xEventGroupCreate();
  qCmd = xQueueCreate(16, sizeof(Command));

  static TaskHandle_t hUI=nullptr, hUtil=nullptr, hMotion=nullptr;
  xTaskCreatePinnedToCore(vTaskUI,     "UI",     4096, nullptr, 2, &hUI,     0);
  xTaskCreatePinnedToCore(vTaskUtil,   "UTIL",   2048, nullptr, 1, &hUtil,   0);
  xTaskCreatePinnedToCore(vTaskMotion, "MOTION", 4096, nullptr, 3, &hMotion, 1);

  OUT_println("\nESP32 Dual-Core listo. Escribe 'help' (USB o BT) para ver comandos.");
}

void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000)); // todo en tareas
}
