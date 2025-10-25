#include <FastAccelStepper.h>

// ===== Pines =====
#define STEP_PIN 18
#define DIR_PIN  19
#define ENDSTOP_PIN 13

// Cambia a 0 si tu endstop es NC
#define ENDSTOP_ACTIVE_LOW 1  

// ===== Motor / Driver =====
FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

// ===== Parámetros =====
float STEPS_PER_MM = 50.0f;   // pasos por mm
int SPEED_HZ = 10000;         // pasos/s
int ACCEL    = 20000;         // pasos/s²

// ===== Homing =====
const bool HOME_TOWARD_NEG = true;    // true = hacia atrás
const float HOME_BACKOFF_MM = 3.0f;   // retrocede un poco al liberar
const int HOME_FAST_HZ = 5000;        // velocidad rápida
const int HOME_SLOW_HZ = 1200;        // velocidad lenta

bool HOMED = false; // bandera de homing completado

String cmd;

// ==================== Funciones ====================

inline long mmToSteps(float mm) {
  return (long)round(mm * STEPS_PER_MM);
}

inline bool endstopPressed() {
  int v = digitalRead(ENDSTOP_PIN);
  return ENDSTOP_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Espera hasta que termine el movimiento
void waitUntilStop() {
  while (stepper->isRunning()) {
    delay(1);
  }
}

// ---------------- HOMING ----------------
void goHome() {
  Serial.println(F("\n=== INICIANDO HOMING ==="));
  HOMED = false;

  // Configuración inicial
  stepper->setAcceleration(ACCEL);
  stepper->setSpeedInHz(HOME_FAST_HZ);

  long direction = HOME_TOWARD_NEG ? -1 : 1;

  // 1) Búsqueda rápida hacia el endstop
  Serial.println(F("→ Búsqueda rápida del endstop..."));
  stepper->move(direction * mmToSteps(1000));  // recorrido máximo
  while (stepper->isRunning()) {
    if (endstopPressed()) {
      stepper->forceStopAndNewPosition(0);  // detener y fijar posición
      break;
    }
  }

  if (!endstopPressed()) {
    Serial.println(F("❌ ERROR: no se detectó el endstop."));
    // 🔁 restaurar velocidad normal antes de salir
    stepper->setSpeedInHz(SPEED_HZ);
    stepper->setAcceleration(ACCEL);
    return;
  }

  // 2) Retroceso corto para liberar el switch
  Serial.println(F("→ Retroceso corto..."));
  stepper->setSpeedInHz(HOME_SLOW_HZ);
  stepper->move(-direction * mmToSteps(HOME_BACKOFF_MM));
  waitUntilStop();
  delay(100);

  // 3) Aproximación lenta al endstop
  Serial.println(F("→ Aproximación lenta..."));
  stepper->move(direction * mmToSteps(HOME_BACKOFF_MM * 2));
  while (stepper->isRunning()) {
    if (endstopPressed()) {
      stepper->forceStopAndNewPosition(0);
      break;
    }
  }

  // 4) Homing completado
  stepper->setCurrentPosition(0);
  HOMED = true;
  Serial.println(F("✅ Homing completado. Posición actual = 0 mm"));

  // 🔁 Restaurar velocidad y aceleración normales
  stepper->setSpeedInHz(SPEED_HZ);
  stepper->setAcceleration(ACCEL);
  Serial.println(F("⚙️ Velocidad restaurada a la normal.\n"));
}

// ---------------- MOVER A 0 ----------------
void goToZero() {
  if (!HOMED) {
    Serial.println(F("⚠️ Primero debes ejecutar 'home'"));
    return;
  }

  Serial.println(F("→ Moviendo a posición 0..."));
  long target = 0 - stepper->getCurrentPosition();
  stepper->move(target);
  waitUntilStop();
  Serial.println(F("OK: en posición 0"));
}

// ---------------- HELP ----------------
void printHelp() {
  Serial.println(F("\nComandos disponibles:"));
  Serial.println(F("  home      -> ejecutar homing (ir al endstop y fijar 0)"));
  Serial.println(F("  g0        -> ir al punto 0"));
  Serial.println(F("  mm=<n>    -> mover n mm"));
  Serial.println(F("  pos?      -> mostrar posición actual"));
  Serial.println(F("  help      -> mostrar esta ayuda\n"));
}

// ---------------- COMANDOS ----------------
void parseCommand(String s) {
  s.trim(); s.toLowerCase();
  if (s == "help") { printHelp(); return; }
  if (s == "home") { goHome(); return; }
  if (s == "g0") { goToZero(); return; }
  if (s == "pos?") {
    long pos = stepper->getCurrentPosition();
    Serial.print(F("Posición actual: "));
    Serial.print(pos);
    Serial.print(F(" pasos ("));
    Serial.print(pos / STEPS_PER_MM, 2);
    Serial.println(F(" mm)"));
    return;
  }

  // Movimiento directo mm=<n>
  if (s.startsWith("mm=")) {
    float mm = s.substring(3).toFloat();
    long steps = mmToSteps(mm);
    Serial.print(F("→ Mover "));
    Serial.print(mm);
    Serial.println(F(" mm"));
    stepper->move(steps);
    waitUntilStop();
    Serial.println(F("OK: movimiento completado"));
    return;
  }

  Serial.println(F("Comando desconocido. Escribe 'help'"));
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("ESP32 + TB6600 con HOMING y ENDSTOP (pin 13)"));
  Serial.println(F("Escribe 'help' para ver comandos.\n"));

  pinMode(ENDSTOP_PIN, INPUT_PULLUP);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("Error al inicializar el motor."));
    while (1);
  }

  // Dirección invertida
  stepper->setDirectionPin(DIR_PIN, false);
  stepper->setAutoEnable(false);

  // Configuración inicial
  stepper->setSpeedInHz(SPEED_HZ);
  stepper->setAcceleration(ACCEL);
}

// ==================== LOOP ====================
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
