#include <FastAccelStepper.h>

// ==== Pines ESP32 → TB6600 (sin EN) ====
// Eje X
#define X_STEP_PIN 25
#define X_DIR_PIN  26
// Eje Y
#define Y_STEP_PIN 18
#define Y_DIR_PIN  19

FastAccelStepperEngine engine;
FastAccelStepper* stepperX = nullptr;
FastAccelStepper* stepperY = nullptr;

// ==== Calibración (pasos por mm) por eje ====
float STEPS_PER_MM_X = 50.0f;
float STEPS_PER_MM_Y = 50.0f;

// ==== Dinámica por eje ====
int  SPEED_HZ_X = 10000;
int  ACCEL_X    = 20000;
int  SPEED_HZ_Y = 10000;
int  ACCEL_Y    = 20000;

// UART
String cmd;

static long mmToSteps(float mm, float steps_per_mm) {
  double steps_d = (double)mm * (double)steps_per_mm;
  return (long) llround(steps_d);
}

void printHelp() {
  Serial.println(F("\nComandos:"));
  Serial.println(F("  x=<n>     | x=<n>mm  | x=<n>cm   -> mueve eje X (cm por defecto)"));
  Serial.println(F("  y=<n>     | y=<n>mm  | y=<n>cm   -> mueve eje Y (cm por defecto)"));
  Serial.println(F("  calx=<ppm>  caly=<ppm>           -> pasos/mm por eje"));
  Serial.println(F("  vx=<steps/s> vy=<steps/s>        -> velocidad por eje"));
  Serial.println(F("  ax=<steps/s^2> ay=<steps/s^2>    -> aceleracion por eje"));
  Serial.println(F("  help"));
  Serial.print  (F("\nX: pasos/mm=")); Serial.print(STEPS_PER_MM_X);
  Serial.print  (F(" V=")); Serial.print(SPEED_HZ_X);
  Serial.print  (F(" A=")); Serial.println(ACCEL_X);
  Serial.print  (F("Y: pasos/mm=")); Serial.print(STEPS_PER_MM_Y);
  Serial.print  (F(" V=")); Serial.print(SPEED_HZ_Y);
  Serial.print  (F(" A=")); Serial.println(ACCEL_Y);
}

bool parseAxisMove(char axis, const String& payload) {
  bool is_mm = false;
  String val = payload;
  String s = payload;
  s.trim(); s.toLowerCase();

  if (s.endsWith("mm")) { is_mm = true;  val = s.substring(0, s.length() - 2); }
  else if (s.endsWith("cm")) { is_mm = false; val = s.substring(0, s.length() - 2); }
  else { is_mm = false; val = s; } // por defecto cm

  float mag = val.toFloat();
  if (!isfinite(mag)) { Serial.println(F("ERR: distancia invalida")); return true; }

  float mm = is_mm ? mag : (mag * 10.0f);

  if (axis == 'x') {
    long steps = mmToSteps(mm, STEPS_PER_MM_X);
    Serial.print(F("X move: ")); Serial.print(mm, 3);
    Serial.print(F(" mm -> ")); Serial.print(steps); Serial.println(F(" pasos"));
    stepperX->move(steps);
    while (stepperX->isRunning()) { }
    Serial.println(F("OK X done"));
    return true;
  } else if (axis == 'y') {
    long steps = mmToSteps(mm, STEPS_PER_MM_Y);
    Serial.print(F("Y move: ")); Serial.print(mm, 3);
    Serial.print(F(" mm -> ")); Serial.print(steps); Serial.println(F(" pasos"));
    stepperY->move(steps);
    while (stepperY->isRunning()) { }
    Serial.println(F("OK Y done"));
    return true;
  }
  return false;
}

bool parseAndExecute(const String& line) {
  String s = line; s.trim(); s.toLowerCase();
  if (s.length() == 0) return false;

  if (s == "help" || s == "?") { printHelp(); return true; }

  // Calibración
  if (s.startsWith("calx=")) {
    float ppm = s.substring(5).toFloat();
    if (ppm > 0 && isfinite(ppm)) { STEPS_PER_MM_X = ppm; stepperX->setSpeedInHz(SPEED_HZ_X); stepperX->setAcceleration(ACCEL_X); Serial.print(F("OK calx=")); Serial.println(STEPS_PER_MM_X); }
    else Serial.println(F("ERR calx"));
    return true;
  }
  if (s.startsWith("caly=")) {
    float ppm = s.substring(5).toFloat();
    if (ppm > 0 && isfinite(ppm)) { STEPS_PER_MM_Y = ppm; stepperY->setSpeedInHz(SPEED_HZ_Y); stepperY->setAcceleration(ACCEL_Y); Serial.print(F("OK caly=")); Serial.println(STEPS_PER_MM_Y); }
    else Serial.println(F("ERR caly"));
    return true;
  }

  // Velocidad / Aceleración
  if (s.startsWith("vx=")) { int v = s.substring(3).toInt(); if (v > 0) { SPEED_HZ_X = v; stepperX->setSpeedInHz(SPEED_HZ_X); Serial.print(F("OK vx=")); Serial.println(v); } else Serial.println(F("ERR vx")); return true; }
  if (s.startsWith("vy=")) { int v = s.substring(3).toInt(); if (v > 0) { SPEED_HZ_Y = v; stepperY->setSpeedInHz(SPEED_HZ_Y); Serial.print(F("OK vy=")); Serial.println(v); } else Serial.println(F("ERR vy")); return true; }
  if (s.startsWith("ax=")) { int a = s.substring(3).toInt(); if (a > 0) { ACCEL_X = a; stepperX->setAcceleration(ACCEL_X); Serial.print(F("OK ax=")); Serial.println(a); } else Serial.println(F("ERR ax")); return true; }
  if (s.startsWith("ay=")) { int a = s.substring(3).toInt(); if (a > 0) { ACCEL_Y = a; stepperY->setAcceleration(ACCEL_Y); Serial.print(F("OK ay=")); Serial.println(a); } else Serial.println(F("ERR ay")); return true; }

  // Movimientos por eje
  if (s.startsWith("x=")) return parseAxisMove('x', s.substring(2));
  if (s.startsWith("y=")) return parseAxisMove('y', s.substring(2));
  if (s.startsWith("x ")) return parseAxisMove('x', s.substring(2));
  if (s.startsWith("y ")) return parseAxisMove('y', s.substring(2));

  Serial.println(F("ERR: comando no reconocido (usa 'help')"));
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println(F("ESP32 + TB6600 (X & Y) - UART distance control"));
  Serial.println(F("Escribe 'help' para ver comandos."));

  engine.init();

  stepperX = engine.stepperConnectToPin(X_STEP_PIN);
  stepperY = engine.stepperConnectToPin(Y_STEP_PIN);
  if (!stepperX || !stepperY) { Serial.println(F("Error: verifica pines STEP")); while (true) { delay(1000); } }

  // X: dirección normal
  stepperX->setDirectionPin(X_DIR_PIN);
  // Y: dirección invertida
  stepperY->setDirectionPin(Y_DIR_PIN, /*dir_high_counts_up=*/false);

  // Sin EN pin
  stepperX->setAutoEnable(false);
  stepperY->setAutoEnable(false);

  stepperX->setSpeedInHz(SPEED_HZ_X);
  stepperX->setAcceleration(ACCEL_X);
  stepperY->setSpeedInHz(SPEED_HZ_Y);
  stepperY->setAcceleration(ACCEL_Y);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd.length() > 0) { parseAndExecute(cmd); cmd = ""; }
    } else {
      cmd += c;
    }
  }
}
