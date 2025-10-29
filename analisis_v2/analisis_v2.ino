#include <FastAccelStepper.h>
/*
P1= x:20.0 ,y:0.0
P2= x:200.0 ,y:0.0
P3= x:200.0 ,y:130.0
P4= x:20.0, y:130.0
P5= x:20.0, y:265.0
P6= x:200.0, y:265.0
*/

// ===== Pines EJE X =====
#define X_STEP_PIN     18
#define X_DIR_PIN      19
#define X_ENDSTOP_PIN  13
#define X_ENDSTOP_ACTIVE_LOW 1   // 1: NO→GND (activo LOW), 0: NC (activo HIGH)

// ===== Pines EJE Y =====
#define Y_STEP_PIN     25
#define Y_DIR_PIN      26
#define Y_ENDSTOP_PIN  27
#define Y_ENDSTOP_ACTIVE_LOW 1   // 1: NO→GND (activo LOW), 0: NC (activo HIGH)

// ===== Ventilador (ON/OFF) =====
#define FAN_PIN           14
#define FAN_ACTIVE_HIGH    1      // HIGH = ON, LOW = OFF

// ===== Límites de recorrido (mm) =====
const float X_MAX_MM = 220.0f;
const float Y_MAX_MM = 320.0f;

// ===== Motores / Driver =====
FastAccelStepperEngine engine;
FastAccelStepper* stepperX = nullptr;
FastAccelStepper* stepperY = nullptr;

// ===== Parámetros por eje =====
float X_STEPS_PER_MM = 40.0f;    // calibrado para GT2 20 dientes
float Y_STEPS_PER_MM = 50.0f;    // ajusta si es distinto

int   X_SPEED_HZ = 10000;
int   X_ACCEL    = 20000;
int   Y_SPEED_HZ = 10000;
int   Y_ACCEL    = 20000;

// ===== Homing =====
const bool X_HOME_TOWARD_NEG = true;
const bool Y_HOME_TOWARD_NEG = true;
const float X_HOME_BACKOFF_MM = 3.0f;
const float Y_HOME_BACKOFF_MM = 3.0f;
const int   X_HOME_FAST_HZ = 5000;
const int   X_HOME_SLOW_HZ = 1200;
const int   Y_HOME_FAST_HZ = 5000;
const int   Y_HOME_SLOW_HZ = 1200;

bool X_HOMED = false;
bool Y_HOMED = false;

// Direcciones invertidas (como pediste)
const bool X_DIR_INVERTED = true;
const bool Y_DIR_INVERTED = true;

// UART
String cmd;

// Ventilador
bool fan_on = false;

// ==================== Utilidades ====================
inline long mmToStepsX(float mm) { return (long)round(mm * X_STEPS_PER_MM); }
inline long mmToStepsY(float mm) { return (long)round(mm * Y_STEPS_PER_MM); }

inline float stepsToMmX(long steps) { return (float)steps / X_STEPS_PER_MM; }
inline float stepsToMmY(long steps) { return (float)steps / Y_STEPS_PER_MM; }

inline float getXmm() { return stepsToMmX(stepperX->getCurrentPosition()); }
inline float getYmm() { return stepsToMmY(stepperY->getCurrentPosition()); }

inline bool endstopXPressed() {
  int v = digitalRead(X_ENDSTOP_PIN);
  return X_ENDSTOP_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}
inline bool endstopYPressed() {
  int v = digitalRead(Y_ENDSTOP_PIN);
  return Y_ENDSTOP_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void waitUntilStop(FastAccelStepper* s) {
  while (s->isRunning()) { delay(1); }
}

// ===== Ventilador ON/OFF =====
void fanWrite(bool on) {
  fan_on = on;
  if (FAN_ACTIVE_HIGH)
    digitalWrite(FAN_PIN, on ? HIGH : LOW);
  else
    digitalWrite(FAN_PIN, on ? LOW : HIGH);
}
void fanOn()  { fanWrite(true);  }
void fanOff() { fanWrite(false); }

// ==================== HOMING BÁSICO ====================
bool doHomeAxis(FastAccelStepper* s,
                bool (*endstopPressed)(),
                bool homeTowardNeg,
                float backoff_mm,
                int fast_hz,
                int slow_hz,
                int speed_normal_hz,
                int accel_normal,
                float steps_per_mm) {
  Serial.println(F("  - Fase 1: búsqueda rápida del endstop..."));
  s->setAcceleration(accel_normal);
  s->setSpeedInHz(fast_hz);

  long direction = homeTowardNeg ? -1 : 1;
  long maxSteps = (long)round(1000.0f * steps_per_mm);
  s->move(direction * maxSteps);

  while (s->isRunning()) {
    if (endstopPressed()) {
      s->forceStopAndNewPosition(0);
      break;
    }
  }

  if (!endstopPressed()) {
    Serial.println(F("ERROR: endstop no detectado."));
    s->setSpeedInHz(speed_normal_hz);
    s->setAcceleration(accel_normal);
    return false;
  }

  // Retroceso
  Serial.println(F("  - Fase 2: retroceso..."));
  s->setSpeedInHz(slow_hz);
  s->move(-direction * (long)round(backoff_mm * steps_per_mm));
  waitUntilStop(s);
  delay(80);

  // Aproximación lenta
  Serial.println(F("  - Fase 3: aproximación lenta..."));
  s->move(direction * (long)round(backoff_mm * 2.0f * steps_per_mm));
  while (s->isRunning()) {
    if (endstopPressed()) {
      s->forceStopAndNewPosition(0);
      break;
    }
  }

  // Restaurar
  s->setCurrentPosition(0);
  s->setSpeedInHz(speed_normal_hz);
  s->setAcceleration(accel_normal);
  Serial.println(F("Homing OK (pos = 0)"));
  return true;
}

// ==================== Límites ====================
bool checkLimitX(float mmTarget) {
  if (!X_HOMED) return true; // antes del homing, no limitar
  if (mmTarget < 0 || mmTarget > X_MAX_MM) {
    Serial.print(F("Límite X excedido: ")); Serial.print(mmTarget);
    Serial.print(F(" mm (rango 0–")); Serial.print(X_MAX_MM); Serial.println(F(" mm)"));
    return false;
  }
  return true;
}
bool checkLimitY(float mmTarget) {
  if (!Y_HOMED) return true;
  if (mmTarget < 0 || mmTarget > Y_MAX_MM) {
    Serial.print(F("Límite Y excedido: ")); Serial.print(mmTarget);
    Serial.print(F(" mm (rango 0")); Serial.print(Y_MAX_MM); Serial.println(F(" mm)"));
    return false;
  }
  return true;
}

// ==================== PRE-DESENGANCHE (5 mm lejos del home) ====================
void preUnlatchX(float travel_mm = 5.0f) {
  int away_dir = X_HOME_TOWARD_NEG ? +1 : -1;
  float cur = getXmm();
  float target = cur + away_dir * travel_mm;
  if (X_HOMED) {
    if (target < 0)        target = 0;
    if (target > X_MAX_MM) target = X_MAX_MM;
  }
  float delta = target - cur;
  long st = mmToStepsX(delta);
  if (st == 0) return;
  Serial.print(F("↪ Desenganche X ")); Serial.print(delta, 2); Serial.println(F(" mm"));
  stepperX->setSpeedInHz(X_SPEED_HZ);
  stepperX->setAcceleration(X_ACCEL);
  stepperX->move(st);
  waitUntilStop(stepperX);
  delay(50);
}

void preUnlatchY(float travel_mm = 5.0f) {
  int away_dir = Y_HOME_TOWARD_NEG ? +1 : -1;
  float cur = getYmm();
  float target = cur + away_dir * travel_mm;
  if (Y_HOMED) {
    if (target < 0)        target = 0;
    if (target > Y_MAX_MM) target = Y_MAX_MM;
  }
  float delta = target - cur;
  long st = mmToStepsY(delta);
  if (st == 0) return;
  Serial.print(F("↪ Desenganche Y ")); Serial.print(delta, 2); Serial.println(F(" mm"));
  stepperY->setSpeedInHz(Y_SPEED_HZ);
  stepperY->setAcceleration(Y_ACCEL);
  stepperY->move(st);
  waitUntilStop(stepperY);
  delay(50);
}

// ===== Wrappers de Homing (con pre-desenganche 5 mm) =====
void homeX() {
  Serial.println(F("\n=== HOMING X ==="));
  preUnlatchX(5.0f);
  X_HOMED = doHomeAxis(stepperX, endstopXPressed, X_HOME_TOWARD_NEG,
                       X_HOME_BACKOFF_MM, X_HOME_FAST_HZ, X_HOME_SLOW_HZ,
                       X_SPEED_HZ, X_ACCEL, X_STEPS_PER_MM);
  if (!X_HOMED) Serial.println(F("⚠️ X: Homing fallido"));
}
void homeY() {
  Serial.println(F("\n=== HOMING Y ==="));
  preUnlatchY(5.0f);
  Y_HOMED = doHomeAxis(stepperY, endstopYPressed, Y_HOME_TOWARD_NEG,
                       Y_HOME_BACKOFF_MM, Y_HOME_FAST_HZ, Y_HOME_SLOW_HZ,
                       Y_SPEED_HZ, Y_ACCEL, Y_STEPS_PER_MM);
  if (!Y_HOMED) Serial.println(F("⚠️ Y: Homing fallido"));
}

// ==================== Movimiento absoluto (SINCRONIZADO) ====================
bool moveToXYAbsSync(float x_mm, float y_mm) {
  // Limites antes de mover
  if (!checkLimitX(x_mm) || !checkLimitY(y_mm)) return false;

  float curx = getXmm();
  float cury = getYmm();
  long dx = mmToStepsX(x_mm - curx);
  long dy = mmToStepsY(y_mm - cury);
  long adx = labs(dx);
  long ady = labs(dy);

  if (adx == 0 && ady == 0) return true; // ya estamos

  // Base común (para que lleguen juntos): usar los mínimos disponibles
  int baseSpeed  = min(X_SPEED_HZ, Y_SPEED_HZ);
  int baseAccel  = min(X_ACCEL,    Y_ACCEL);

  long maxSteps = max(adx, ady);
  // Evitar 0 y valores demasiado bajos
  auto scaled = [&](long a, int base)->int {
    if (a == 0) return 0;
    long v = lround((double)base * (double)a / (double)maxSteps);
    if (v < 100) v = 100; // umbral mínimo razonable
    return (int)v;
  };

  int vx = scaled(adx, baseSpeed);
  int vy = scaled(ady, baseSpeed);
  int ax = scaled(adx, baseAccel);
  int ay = scaled(ady, baseAccel);

  // Guardar parámetros para restaurar luego
  int old_vx = X_SPEED_HZ, old_ax = X_ACCEL;
  int old_vy = Y_SPEED_HZ, old_ay = Y_ACCEL;

  // Aplicar parámetros sincronizados
  if (adx > 0) { stepperX->setSpeedInHz(vx); stepperX->setAcceleration(ax); }
  if (ady > 0) { stepperY->setSpeedInHz(vy); stepperY->setAcceleration(ay); }

  // Lanzar ambos movimientos casi simultáneamente
  if (adx > 0) stepperX->move(dx);
  if (ady > 0) stepperY->move(dy);

  // Esperar hasta que ambos terminen
  while ((adx > 0 && stepperX->isRunning()) || (ady > 0 && stepperY->isRunning())) {
    delay(1);
  }

  // Restaurar parámetros normales
  stepperX->setSpeedInHz(old_vx);
  stepperX->setAcceleration(old_ax);
  stepperY->setSpeedInHz(old_vy);
  stepperY->setAcceleration(old_ay);

  return true;
}

// Helpers absolutos por eje (se mantienen por si quieres usarlos)
bool moveToXAbs(float x_mm) {
  if (!checkLimitX(x_mm)) return false;
  float cur = getXmm();
  long st = mmToStepsX(x_mm - cur);
  if (st == 0) return true;
  stepperX->move(st);
  waitUntilStop(stepperX);
  return true;
}
bool moveToYAbs(float y_mm) {
  if (!checkLimitY(y_mm)) return false;
  float cur = getYmm();
  long st = mmToStepsY(y_mm - cur);
  if (st == 0) return true;
  stepperY->move(st);
  waitUntilStop(stepperY);
  return true;
}

// ==================== Ir a cero ====================
void goToZeroX() {
  if (!X_HOMED) { Serial.println(F("⚠️ Primero 'homex'")); return; }
  moveToXAbs(0.0f);
  Serial.println(F("OK X=0"));
}
void goToZeroY() {
  if (!Y_HOMED) { Serial.println(F("⚠️ Primero 'homey'")); return; }
  moveToYAbs(0.0f);
  Serial.println(F("OK Y=0"));
}

// ==================== ANALISIS ====================
void doAnalisis() {
  Serial.println(F("\n=== ANALISIS: homing + recorrido de puntos (simultáneo) ==="));

  // Homing con pre-desenganche ya integrado
  homeX();
  homeY();
  if (!X_HOMED || !Y_HOMED) {
    Serial.println(F("No se pudo completar homing. Abortando analisis."));
    return;
  }

  // Puntos absolutos
  const uint8_t NPTS = 6;
  const float PX[NPTS] = { 20.0f, 200.0f, 200.0f,  20.0f,  20.0f, 200.0f };
  const float PY[NPTS] = {  0.0f,   0.0f, 130.0f, 130.0f, 265.0f, 265.0f };

  for (uint8_t i = 0; i < NPTS; ++i) {
    Serial.print(F("→ Punto P")); Serial.print(i+1);
    Serial.print(F(" = (")); Serial.print(PX[i],1); Serial.print(F(", "));
    Serial.print(PY[i],1); Serial.println(F(") mm"));

    if (!moveToXYAbsSync(PX[i], PY[i])) {
      Serial.println(F("Movimiento cancelado por límite. Abortando analisis."));
      return;
    }

    delay(2000); // 2 s en cada punto
  }

  Serial.println(F("ANALISIS completado."));
}

// ==================== Ir directo a P1..P6 ====================
void gotoPoint(uint8_t idx1based) {
  const float PX[6] = { 20.0f, 200.0f, 200.0f,  20.0f,  20.0f, 200.0f };
  const float PY[6] = {  0.0f,   0.0f, 130.0f, 130.0f, 265.0f, 265.0f };

  if (idx1based < 1 || idx1based > 6) return;
  uint8_t i = idx1based - 1;

  if (!X_HOMED || !Y_HOMED) {
    Serial.println(F("ℹ️ No homed: realizando homing antes de ir al punto..."));
    homeX();
    homeY();
    if (!X_HOMED || !Y_HOMED) {
      Serial.println(F("Homing falló. No se puede ir al punto."));
      return;
    }
  }

  Serial.print(F("→ Ir a P")); Serial.print(idx1based);
  Serial.print(F(" (")); Serial.print(PX[i],1); Serial.print(F(", "));
  Serial.print(PY[i],1); Serial.println(F(") mm"));

  if (!moveToXYAbsSync(PX[i], PY[i])) {
    Serial.println(F("Movimiento cancelado por límite."));
  } else {
    Serial.println(F("OK punto alcanzado."));
  }
}

// ==================== Ayuda ====================
void printHelp() {
  Serial.println(F("\nComandos:"));
  Serial.println(F("  analisis    -> homing y recorrido P1..P6 (2s por punto, XY simultáneo)"));
  Serial.println(F("  p1..p6      -> ir directo al punto P1..P6 (XY simultáneo)"));
  Serial.println(F("  home        -> homing X y luego Y (con pre-desenganche 5mm)"));
  Serial.println(F("  homex       -> homing solo X (con pre-desenganche)"));
  Serial.println(F("  homey       -> homing solo Y (con pre-desenganche)"));
  Serial.println(F("  g0          -> ir X=0 luego Y=0"));
  Serial.println(F("  g0x         -> ir X=0"));
  Serial.println(F("  g0y         -> ir Y=0"));
  Serial.println(F("  mmx=<n>     -> mover X n mm (relativo)"));
  Serial.println(F("  mmy=<n>     -> mover Y n mm (relativo)"));
  Serial.println(F("  pos?        -> mostrar posiciones X/Y"));
  Serial.println(F("  fanon       -> encender ventilador"));
  Serial.println(F("  fanoff      -> apagar ventilador"));
  Serial.println(F("  fan?        -> estado ventilador"));
  Serial.println(F("  help        -> esta ayuda\n"));
}

// ==================== Parser ====================
void parseCommand(String s) {
  s.trim(); s.toLowerCase();
  if (s == "help" || s == "?") { printHelp(); return; }

  if (s == "analisis") { doAnalisis(); return; }

  // Puntos directos
  if (s == "p1") { gotoPoint(1); return; }
  if (s == "p2") { gotoPoint(2); return; }
  if (s == "p3") { gotoPoint(3); return; }
  if (s == "p4") { gotoPoint(4); return; }
  if (s == "p5") { gotoPoint(5); return; }
  if (s == "p6") { gotoPoint(6); return; }

  if (s == "homex") { homeX(); return; }
  if (s == "homey") { homeY(); return; }
  if (s == "home")  { homeX(); if (X_HOMED) homeY(); return; }

  if (s == "g0x") { goToZeroX(); return; }
  if (s == "g0y") { goToZeroY(); return; }
  if (s == "g0")  { goToZeroX(); goToZeroY(); return; }

  if (s == "pos?") {
    Serial.print(F("X: ")); Serial.print(getXmm(), 2); Serial.println(F(" mm"));
    Serial.print(F("Y: ")); Serial.print(getYmm(), 2); Serial.println(F(" mm"));
    return;
  }

  // Movimientos relativos individuales (se mantienen como antes)
  if (s.startsWith("mmx=")) {
    float mm = s.substring(4).toFloat();
    float destino = getXmm() + mm;
    if (!checkLimitX(destino)) return;
    long st = mmToStepsX(mm);
    Serial.print(F("→ Mover X ")); Serial.print(mm); Serial.println(F(" mm"));
    stepperX->move(st);
    waitUntilStop(stepperX);
    Serial.println(F("OK X movido"));
    return;
  }
  if (s.startsWith("mmy=")) {
    float mm = s.substring(4).toFloat();
    float destino = getYmm() + mm;
    if (!checkLimitY(destino)) return;
    long st = mmToStepsY(mm);
    Serial.print(F("→ Mover Y ")); Serial.print(mm); Serial.println(F(" mm"));
    stepperY->move(st);
    waitUntilStop(stepperY);
    Serial.println(F("OK Y movido"));
    return;
  }

  // Ventilador
  if (s == "fanon")  { fanOn();  Serial.println(F("Ventilador: ON"));  return; }
  if (s == "fanoff") { fanOff(); Serial.println(F("Ventilador: OFF")); return; }
  if (s == "fan?")   {
    Serial.print(F("Ventilador: ")); Serial.println(fan_on ? F("ON") : F("OFF"));
    return;
  }

  Serial.println(F("Comando desconocido. Escribe 'help'"));
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println(F("ESP32 + 2xTB6600 + Límites + Ventilador ON/OFF + ANALISIS + P1..P6 (XY simultáneo)"));
  Serial.println(F("Comandos: 'analisis', 'p1..p6', 'home', 'mmx=', 'mmy=', 'pos?', 'fanon', 'fanoff'\n"));

  pinMode(X_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(FAN_PIN, OUTPUT);
  fanOff();

  engine.init();

  // ---- Eje X ----
  stepperX = engine.stepperConnectToPin(X_STEP_PIN);
  if (!stepperX) { Serial.println(F("Error al inicializar X")); while (1) {} }
  stepperX->setDirectionPin(X_DIR_PIN, false); // dirección invertida
  stepperX->setAutoEnable(false);
  stepperX->setSpeedInHz(X_SPEED_HZ);
  stepperX->setAcceleration(X_ACCEL);

  // ---- Eje Y ----
  stepperY = engine.stepperConnectToPin(Y_STEP_PIN);
  if (!stepperY) { Serial.println(F("Error al inicializar Y")); while (1) {} }
  stepperY->setDirectionPin(Y_DIR_PIN, false); // dirección invertida
  stepperY->setAutoEnable(false);
  stepperY->setSpeedInHz(Y_SPEED_HZ);
  stepperY->setAcceleration(Y_ACCEL);
}

// ==================== Loop ====================
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd.length() > 0) {
        parseCommand(cmd);
        cmd = "";
      }
    } else {
      cmd += c;
    }
  }
}
