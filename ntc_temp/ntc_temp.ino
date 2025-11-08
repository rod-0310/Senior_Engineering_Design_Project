#include <SmoothThermistor.h>

// ================== AJUSTES PARA ESP32 ==================
// Usaremos un pin ADC de ESP32 (ADC1) seguro para lecturas analógicas.
// Opciones típicas: 32, 33, 34, 35, 36, 39 (34-39 son solo entrada).
#define PIN_THERMISTOR 34

// ESP32 tiene ADC de 12 bits por defecto (0–4095).
// Mantengo tus parámetros del NTC y el promedio de 10 muestras.
SmoothThermistor thermistor(
  PIN_THERMISTOR,        // Pin analógico (ESP32)
  ADC_SIZE_12_BIT,       // Resolución ADC ESP32 (0–4095)
  100000,                // Resistencia nominal del NTC (100kΩ @ 25°C)
  10000,                 // Resistencia serie (10kΩ)
  3950,                  // Coeficiente Beta (ajusta si conoces el exacto)
  25,                    // Temp. nominal (°C)
  10                     // Muestras por lectura (promedio)
);

void setup() {
  Serial.begin(115200);
  delay(100);

  // ------- Configuración específica del ADC en ESP32 -------
  // Aseguramos 12 bits en el ADC
  analogReadResolution(12);

  // Atenuación 11 dB -> rango aprox. 0–3.3 V (ideal para divisor a 3.3 V)
  // (Disponibles: ADC_0db, ADC_2_5db, ADC_6db, ADC_11db)
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_THERMISTOR, ADC_11db);

  // En ESP32 no hay AREF como en AVR. Déjalo en false.
  // (Si alguna vez alimentas el divisor con 3.3 V externos, sigue en false).
  thermistor.useAREF(false);

  Serial.println("Iniciando lectura de NTC 100k en ESP32...");
}

void loop() {
  float tempC = thermistor.temperature();

  Serial.print("Temperatura: ");
  Serial.print(tempC, 2);
  Serial.println(" °C");

  delay(1000);
}
