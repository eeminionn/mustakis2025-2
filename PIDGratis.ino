#include <QTRSensors.h>
#include <Wire.h>

// Pines ESP32
#define BOTON 12
#define LED   2

// PID
float Kp = 0.13;
float Ki = 0.0;
float Kd = 0.5;

int lastError = 0;
int integral  = 0;

const int freq = 5000;
const int resolution = 8;


// Velocidad base
const int velocidadBase = 50;

// Sensores QTR
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

QTRSensors qtr;

// ────────────────────────────────────────────────────────────────
void inicializarMotores();
void Motor(int velIzq, int velDer);
// ────────────────────────────────────────────────────────────────

// PID sobre la posición [0..7000] (8 sensores → 7 espacios de 1000)
void PID(uint16_t position) {
  int error = position - 3500;    // centro = 3500
  integral  += error;
  int derivative = error - lastError;
  int output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int vIzq = constrain(velocidadBase + output, 0, 255);
  int vDer = constrain(velocidadBase - output, 0, 255);
  Motor(vIzq, vDer);
}

void setup() {
  Serial.begin(115200);
  inicializarMotores();

  pinMode(LED, OUTPUT);
  pinMode(BOTON, INPUT);

  // Config QTR (analógico, mismos pines)
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 36, 39, 34, 35, 32, 33, 25, 26 }, SensorCount);
  qtr.setEmitterPin(27);

  Serial.println("Esperando botón para calibrar");
  while (digitalRead(BOTON) == LOW) delay(10);

  Serial.println("Calibrando sensores...");
  for (uint16_t i = 0; i < 150; i++) qtr.calibrate();

  Serial.println("Listo. Esperando para comenzar...");
  while (digitalRead(BOTON) == LOW) delay(10);
  delay(500);
}

void loop() {
  // Leer crudos
  qtr.read(sensorValues);

  // Umbral: todo lo < 3500 lo anulamos
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 3500) sensorValues[i] = 0;
  }

  // Posición ponderada manual
  uint32_t sumaPesada = 0;
  uint32_t sumaTotal  = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    sumaPesada += (uint32_t)sensorValues[i] * (i * 1000);
    sumaTotal  += sensorValues[i];
  }

  // GAP: todo blanco → avanza recto con base y no acumules integral
  if (sumaTotal == 0) {
    Motor(velocidadBase, velocidadBase);
    integral  = 0;
    lastError = 0;
    return;
  }

  uint16_t position = sumaPesada / sumaTotal;

  // Control PID
  PID(position);
}
