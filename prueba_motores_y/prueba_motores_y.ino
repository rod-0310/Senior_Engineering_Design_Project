#include <FastAccelStepper.h>

// ESP32 → TB6600 (solo STEP y DIR; PUL-/DIR- a GND común)
#define STEP_PIN 18
#define DIR_PIN  19

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

// ===== Calibración y dinámica =====
// Con 16T + GT2 + 1/8 microstep ≈ 50 pasos/mm (ajústalo si hace falta).
float STEPS_PER_MM = 50.0f;

int  SPEED_HZ = 10000;   // pasos/seg
int  ACCEL     = 20000; // pasos/seg^2

// ===== UART =====
String cmd;

void printHelp() {
  Serial.println(F("\nComandos:"));
  Serial.println(F("  <n>           -> mueve n cm (ej: 5  / -3.2)"));
  Serial.println(F("  <n>mm         -> mueve n mm (ej: 25mm / -12mm)"));
  Serial.println(F("  cm=<n>        -> mueve n cm"));
  Serial.println(F("  mm=<n>        -> mueve n mm"));
  Serial.println(F("  cal=<ppm>     -> set pasos/mm (ej: cal=50)"));
  Serial.println(F("  v=<steps/s>   -> set velocidad (ej: v=6000)"));
  Serial.println(F("  a=<steps/s^2> -> set aceleracion (ej: a=20000)"));
  Serial.println(F("  help          -> esta ayuda\n"));
  Serial.print (F("Actual: pasos/mm=")); Serial.print(STEPS_PER_MM);
  Serial.print (F("  V=")); Serial.print(SPEED_HZ);
  Serial.print (F(" steps/s  A=")); Serial.print(ACCEL);
  Serial.println(F(" steps/s^2"));
}

long mmToSteps(float mm) {
  // redondeo al entero más cercano
  double steps_d = (double)mm * (double)STEPS_PER_MM;
  long steps = (long) llround(steps_d);
  return steps;
}

bool parseAndExecute(const String& line) {
  String s = line;
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return false;

  // --- help ---
  if (s == "help" || s == "?") {
    printHelp();
    return true;
  }

  // --- cal=XX (pasos/mm) ---
  if (s.startsWith("cal=")) {
    float ppm = s.substring(4).toFloat();
    if (ppm > 0.0f && isfinite(ppm)) {
      STEPS_PER_MM = ppm;
      stepper->setSpeedInHz(SPEED_HZ);
      stepper->setAcceleration(ACCEL);
      Serial.print(F("OK cal: pasos/mm=")); Serial.println(STEPS_PER_MM);
    } else {
      Serial.println(F("ERR cal: valor invalido"));
    }
    return true;
  }

  // --- v=XXXX (steps/s) ---
  if (s.startsWith("v=")) {
    int v = s.substring(2).toInt();
    if (v > 0) {
      SPEED_HZ = v;
      stepper->setSpeedInHz(SPEED_HZ);
      Serial.print(F("OK V=")); Serial.println(SPEED_HZ);
    } else {
      Serial.println(F("ERR v: valor invalido"));
    }
    return true;
  }

  // --- a=XXXX (steps/s^2) ---
  if (s.startsWith("a=")) {
    int a = s.substring(2).toInt();
    if (a > 0) {
      ACCEL = a;
      stepper->setAcceleration(ACCEL);
      Serial.print(F("OK A=")); Serial.println(ACCEL);
    } else {
      Serial.println(F("ERR a: valor invalido"));
    }
    return true;
  }

  // --- mm=XX / cm=XX / sufijos mm/cm / número "pelado" (cm por defecto) ---
  bool is_mm = false;
  String val = s;

  if (s.startsWith("mm=")) {
    is_mm = true;
    val = s.substring(3);
  } else if (s.startsWith("cm=")) {
    is_mm = false;
    val = s.substring(3);
  } else if (s.endsWith("mm")) {
    is_mm = true;
    val = s.substring(0, s.length() - 2);
  } else if (s.endsWith("cm")) {
    is_mm = false;
    val = s.substring(0, s.length() - 2);
  } else {
    // sin sufijo ni prefijo => por defecto interpretamos en cm
    is_mm = false;
    val = s;
  }

  float mag = val.toFloat();
  if (!isfinite(mag)) {
    Serial.println(F("ERR: distancia invalida"));
    return true;
  }

  float mm = is_mm ? mag : (mag * 10.0f);
  long steps = mmToSteps(mm);

  Serial.print(F("Move: "));
  Serial.print(mm, 3); Serial.print(F(" mm  -> "));
  Serial.print(steps); Serial.println(F(" pasos"));

  stepper->move(steps);
  while (stepper->isRunning()) {
    // Espera activa breve; no uses delays largos
    // (FastAccelStepper maneja el temporizado en ISR)
  }
  Serial.println(F("OK done"));
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println(F("ESP32 + TB6600 UART distance control"));
  Serial.println(F("Escribe 'help' para ver comandos."));

  engine.init();

  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("Error: STEP_PIN invalido"));
    while (true) { delay(1000); }
  }
  stepper->setDirectionPin(DIR_PIN);

  // Sin EN pin (driver habilitado fijo externamente)
  stepper->setAutoEnable(false);

  stepper->setSpeedInHz(SPEED_HZ);
  stepper->setAcceleration(ACCEL);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd.length() > 0) {
        parseAndExecute(cmd);
        cmd = "";
      }
    } else {
      cmd += c;
    }
  }
}
