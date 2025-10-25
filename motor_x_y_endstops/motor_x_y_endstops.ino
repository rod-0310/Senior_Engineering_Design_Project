#include <FastAccelStepper.h>

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
float X_STEPS_PER_MM = 40.0f;    // ✅ calibrado para GT2 20 dientes
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

// ==================== HOMING ====================
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
    Serial.println(F("  ❌ ERROR: endstop no detectado."));
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
  Serial.println(F("  ✅ Homing OK (pos = 0)"));
  return true;
}

// ===== Wrappers de Homing =====
void homeX() {
  Serial.println(F("\n=== HOMING X ==="));
  X_HOMED = doHomeAxis(stepperX, endstopXPressed, X_HOME_TOWARD_NEG,
                       X_HOME_BACKOFF_MM, X_HOME_FAST_HZ, X_HOME_SLOW_HZ,
                       X_SPEED_HZ, X_ACCEL, X_STEPS_PER_MM);
  if (!X_HOMED) Serial.println(F("⚠️ X: Homing fallido"));
}
void homeY() {
  Serial.println(F("\n=== HOMING Y ==="));
  Y_HOMED = doHomeAxis(stepperY, endstopYPressed, Y_HOME_TOWARD_NEG,
                       Y_HOME_BACKOFF_MM, Y_HOME_FAST_HZ, Y_HOME_SLOW_HZ,
                       Y_SPEED_HZ, Y_ACCEL, Y_STEPS_PER_MM);
  if (!Y_HOMED) Serial.println(F("⚠️ Y: Homing fallido"));
}

// ===== Ir a cero =====
void goToZeroX() {
  if (!X_HOMED) { Serial.println(F("⚠️ Primero 'homex'")); return; }
  long delta = 0 - stepperX->getCurrentPosition();
  Serial.println(F("→ X a 0..."));
  stepperX->move(delta);
  waitUntilStop(stepperX);
  Serial.println(F("OK X=0"));
}
void goToZeroY() {
  if (!Y_HOMED) { Serial.println(F("⚠️ Primero 'homey'")); return; }
  long delta = 0 - stepperY->getCurrentPosition();
  Serial.println(F("→ Y a 0..."));
  stepperY->move(delta);
  waitUntilStop(stepperY);
  Serial.println(F("OK Y=0"));
}

// ==================== Lógica de Límites ====================
bool checkLimitX(float mmTarget) {
  if (!X_HOMED) return true; // antes del homing, no limitar
  if (mmTarget < 0 || mmTarget > X_MAX_MM) {
    Serial.print(F("🚫 Límite X excedido: ")); Serial.print(mmTarget);
    Serial.print(F(" mm (rango 0–")); Serial.print(X_MAX_MM); Serial.println(F(" mm)"));
    return false;
  }
  return true;
}

bool checkLimitY(float mmTarget) {
  if (!Y_HOMED) return true;
  if (mmTarget < 0 || mmTarget > Y_MAX_MM) {
    Serial.print(F("🚫 Límite Y excedido: ")); Serial.print(mmTarget);
    Serial.print(F(" mm (rango 0–")); Serial.print(Y_MAX_MM); Serial.println(F(" mm)"));
    return false;
  }
  return true;
}

// ==================== Ayuda ====================
void printHelp() {
  Serial.println(F("\nComandos:"));
  Serial.println(F("  home        -> homing X y luego Y"));
  Serial.println(F("  homex       -> homing solo X"));
  Serial.println(F("  homey       -> homing solo Y"));
  Serial.println(F("  g0          -> ir X=0 luego Y=0"));
  Serial.println(F("  g0x         -> ir X=0"));
  Serial.println(F("  g0y         -> ir Y=0"));
  Serial.println(F("  mmx=<n>     -> mover X n mm"));
  Serial.println(F("  mmy=<n>     -> mover Y n mm"));
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

  if (s == "homex") { homeX(); return; }
  if (s == "homey") { homeY(); return; }
  if (s == "home")  { homeX(); if (X_HOMED) homeY(); return; }

  if (s == "g0x") { goToZeroX(); return; }
  if (s == "g0y") { goToZeroY(); return; }
  if (s == "g0")  { goToZeroX(); goToZeroY(); return; }

  if (s == "pos?") {
    long px = stepperX->getCurrentPosition();
    long py = stepperY->getCurrentPosition();
    Serial.print(F("X: ")); Serial.print(px / X_STEPS_PER_MM, 2); Serial.println(F(" mm"));
    Serial.print(F("Y: ")); Serial.print(py / Y_STEPS_PER_MM, 2); Serial.println(F(" mm"));
    return;
  }

  if (s.startsWith("mmx=")) {
    float mm = s.substring(4).toFloat();
    float posActual = stepperX->getCurrentPosition() / X_STEPS_PER_MM;
    float destino = posActual + mm;
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
    float posActual = stepperY->getCurrentPosition() / Y_STEPS_PER_MM;
    float destino = posActual + mm;
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
  Serial.println(F("ESP32 + 2xTB6600 + Límites físicos + Ventilador ON/OFF"));
  Serial.println(F("Comandos: 'home', 'mmx=', 'mmy=', 'pos?', 'fanon', 'fanoff'\n"));

  pinMode(X_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(FAN_PIN, OUTPUT);
  fanOff();

  engine.init();

  // ---- Eje X ----
  stepperX = engine.stepperConnectToPin(X_STEP_PIN);
  if (!stepperX) { Serial.println(F("Error al inicializar X")); while (1) {} }
  stepperX->setDirectionPin(X_DIR_PIN, false);
  stepperX->setAutoEnable(false);
  stepperX->setSpeedInHz(X_SPEED_HZ);
  stepperX->setAcceleration(X_ACCEL);

  // ---- Eje Y ----
  stepperY = engine.stepperConnectToPin(Y_STEP_PIN);
  if (!stepperY) { Serial.println(F("Error al inicializar Y")); while (1) {} }
  stepperY->setDirectionPin(Y_DIR_PIN, false);
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
