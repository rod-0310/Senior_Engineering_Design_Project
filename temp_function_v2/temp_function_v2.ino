#include <Arduino.h>
#include <SmoothThermistor.h>

// ================== NTC en ESP32 (tu base) ==================
#define PIN_THERMISTOR 34

SmoothThermistor thermistor(
  PIN_THERMISTOR,        // Pin analógico (ESP32)
  ADC_SIZE_12_BIT,       // Resolución ADC ESP32 (0–4095)
  100000,                // NTC 100kΩ @ 25°C
  10000,                 // Resistencia serie 10kΩ
  3950,                  // Beta
  25,                    // Temp nominal (°C)
  10                     // Muestras por lectura
);

// ================== Dimmer de un solo pin (burst-fire) ==================
#define PIN_DIMMER 21          // Entrada lógica del módulo dimmer (PWM/IN)
#define WINDOW_MS 1000UL       // Ventana de control (1 s)
volatile uint8_t dimmer_percent = 100;  // 0–100%

// Relojes
unsigned long window_start_ms = 0;
unsigned long last_temp_ms    = 0;

// (opcional) límites de seguridad
const float TEMP_MAX_C = 60.0;   // apaga si superas esta temp

void setup() {
  Serial.begin(115200);
  delay(100);

  // ------- ADC para NTC -------
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_THERMISTOR, ADC_11db);
  thermistor.useAREF(false);

  // ------- Dimmer (un solo pin) -------
  pinMode(PIN_DIMMER, OUTPUT);
  digitalWrite(PIN_DIMMER, LOW);

  window_start_ms = millis();
  last_temp_ms    = millis();

  Serial.println(F("NTC 100k + Dimmer de un solo pin (burst-fire) listo."));
  Serial.println(F("Comandos: D0..D100 para fijar potencia (%). Ej: D75"));
}

void loop() {
  const unsigned long now = millis();

  // ===== Control de potencia tipo burst-fire =====
  // - Dentro de cada ventana de 1 s, mantener la salida HIGH el % indicado
  //   y LOW el resto. El SSR de cruce por cero hará los cambios en los cruces.
  if (now - window_start_ms >= WINDOW_MS) {
    window_start_ms = now;  // nueva ventana
  }
  // Duración ON para esta ventana
  unsigned long on_ms = (WINDOW_MS * (unsigned long)dimmer_percent) / 100UL;

  // Estado del pin dentro de la ventana
  if (now - window_start_ms < on_ms) {
    digitalWrite(PIN_DIMMER, HIGH);
  } else {
    digitalWrite(PIN_DIMMER, LOW);
  }

  // ===== Lectura de temperatura (cada 5 s) =====
  if (now - last_temp_ms >= 5000UL) {
    last_temp_ms = now;
    float tempC = thermistor.temperature();
    Serial.print("Temperatura: ");
    Serial.print(tempC, 2);
    Serial.print(" °C  |  Dimmer: ");
    Serial.print(dimmer_percent);
    Serial.println(" %");

    // Seguridad simple
    if (!isnan(tempC) && tempC > TEMP_MAX_C) {
      dimmer_percent = 0; // apaga
      Serial.println(F("# Seguridad: temperatura alta, dimmer = 0%"));
    }
  }

  // ===== Comando por Serial: D0..D100 =====
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() >= 2 && (s[0] == 'D' || s[0] == 'd')) {
      int v = s.substring(1).toInt();
      if (v < 0) v = 0;
      if (v > 100) v = 100;
      dimmer_percent = (uint8_t)v;
      Serial.print(F("Nuevo dimmer: "));
      Serial.print(dimmer_percent);
      Serial.println(F(" %"));
    }
  }

  // (no usar delay largo: dejamos el bucle libre para mantener el burst)
  // Si quieres "descansar" un poco la CPU:
  delay(1);
}
