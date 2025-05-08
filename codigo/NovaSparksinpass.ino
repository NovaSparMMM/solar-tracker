// Solar Tracker completo con medición de voltaje y envío a Adafruit IO
#include <AccelStepper.h>
#include "AdafruitIO_WiFi.h"
#include "AdafruitIO_MQTT.h"

// ————— CONFIGURACIÓN WIFI & ADAGIO —————
#define IO_USERNAME    "YOUR_IO_USERNAME"
#define IO_KEY         "YOUR_IO_KEY"
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASS      "YOUR_WIFI_PASS"

// ————— Adafruit IO —————
AdafruitIO_WiFi io("youname", "yourcode", "name", "pass");
AdafruitIO_Feed *voltageFeed = io.feed("voltage");

// ————— Pines LDR —————
#define LDR_TOP_LEFT     32
#define LDR_TOP_RIGHT    34
#define LDR_BOTTOM_LEFT  33
#define LDR_BOTTOM_RIGHT 35

// ————— Fin de carrera eje X —————
#define ENDSTOP_LEFT    4
#define ENDSTOP_RIGHT   5

// ————— Pines stepper X (horizontal) —————
#define M1 14
#define M2 27
#define M3 26
#define M4 25

// ————— Pines stepper Y (vertical) —————
#define VERT_M1 18
#define VERT_M2 19
#define VERT_M3 21
#define VERT_M4 22

// ————— Sensor de voltaje —————
#define VOLTAGE_PIN     36      // ADC1_CH0 en ESP32
// Factor de divisor: ajusta según tu resistencia; p.ej. 6 para 0-25V → 0-3.3V
#define VOLTAGE_DIVIDER 6.0

// ————— Parámetros de control —————
const int SAMPLE_COUNT   = 5;    // muestras para suavizado
const int START_THRESH   = 100;  // iniciar movimiento
const int STOP_THRESH    = 50;   // detener movimiento (histeresis)
const int MIN_STEP       = 5;    // paso mínimo
const int MAX_STEP       = 30;   // paso máximo
const int MAX_SPEED      = 10500; // velocidad máxima motor
const int ACCEL          = 8800;  // aceleración motor
const unsigned long PUBLISH_INTERVAL = 10000; // ms entre envíos de voltaje

// ————— Buffers de suavizado —————
int bufTL[SAMPLE_COUNT], bufTR[SAMPLE_COUNT], bufBL[SAMPLE_COUNT], bufBR[SAMPLE_COUNT];
int bufIdx = 0;

// ————— Objetos Stepper —————
AccelStepper stepperX(AccelStepper::HALF4WIRE, M1, M3, M2, M4);
AccelStepper stepperY(AccelStepper::HALF4WIRE, VERT_M1, VERT_M3, VERT_M2, VERT_M4);

// Temporizador para publicación
unsigned long lastPublish = 0;

// ————— Función de suavizado (promedio móvil) —————
int smooth(int *buf, int val) {
  buf[bufIdx] = val;
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) sum += buf[i];
  return sum / SAMPLE_COUNT;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("🔧 Solar Tracker + Voltaje a Adafruit IO");

  // Inicializar buffers con lectura inicial
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    bufTL[i] = analogRead(LDR_TOP_LEFT);
    bufTR[i] = analogRead(LDR_TOP_RIGHT);
    bufBL[i] = analogRead(LDR_BOTTOM_LEFT);
    bufBR[i] = analogRead(LDR_BOTTOM_RIGHT);
  }

  // Configurar finales de carrera
  pinMode(ENDSTOP_LEFT, INPUT_PULLUP);
  pinMode(ENDSTOP_RIGHT, INPUT_PULLUP);

  // Configurar steppers
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperX.setAcceleration(ACCEL);
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperY.setAcceleration(ACCEL);

  // Iniciar Adafruit IO
  io.connect();
  // Esperar conexión
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("✔️ Conectado a Adafruit IO");

  lastPublish = millis();
}

void loop() {
  io.run(); // Mantener la conexión MQTT

  // ————— LECTURA Y CONTROL TRACKER —————
  int tl = smooth(bufTL, analogRead(LDR_TOP_LEFT));
  int tr = smooth(bufTR, analogRead(LDR_TOP_RIGHT));
  int bl = smooth(bufBL, analogRead(LDR_BOTTOM_LEFT));
  int br = smooth(bufBR, analogRead(LDR_BOTTOM_RIGHT));
  bufIdx = (bufIdx + 1) % SAMPLE_COUNT;

  int avgTop    = (tl + tr) / 2;
  int avgBottom = (bl + br) / 2;
  int avgLeft   = (tl + bl) / 2;
  int avgRight  = (tr + br) / 2;

  int diffVert  = avgTop - avgBottom;
  int diffHoriz = avgRight - avgLeft;

  // Control vertical
  if (abs(diffVert) > START_THRESH) {
    int dirY = (diffVert > 0) ? +1 : -1;
    int stepY = map(abs(diffVert), START_THRESH, 1023, MIN_STEP, MAX_STEP);
    stepY = constrain(stepY, MIN_STEP, MAX_STEP);
    moveVertical(dirY * stepY);
  } else if (abs(diffVert) < STOP_THRESH) {
    stepperY.stop();
  }

  // Control horizontal
  if (abs(diffHoriz) > START_THRESH) {
    int dirX = (diffHoriz > 0) ? +1 : -1;
    int stepX = map(abs(diffHoriz), START_THRESH, 1023, MIN_STEP, MAX_STEP);
    stepX = constrain(stepX, MIN_STEP, MAX_STEP);
    if ((dirX < 0 && digitalRead(ENDSTOP_LEFT) == HIGH) ||
        (dirX > 0 && digitalRead(ENDSTOP_RIGHT)== HIGH)) {
      moveHorizontal(dirX * stepX);
    } else {
      stepperX.stop();
    }
  } else if (abs(diffHoriz) < STOP_THRESH) {
    stepperX.stop();
  }

  stepperX.run();
  stepperY.run();

  // Debug tracker
  Serial.printf("ΔV: %4d | ΔH: %4d → X:%4ld Y:%4ld\n",
                diffVert, diffHoriz,
                stepperX.currentPosition(),
                stepperY.currentPosition());

  // ————— LECTURA Y ENVÍO DE VOLTAJE —————
  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    int raw = analogRead(VOLTAGE_PIN);
    float measuredV = (raw * 3.3 / 4095.0) * VOLTAGE_DIVIDER;
    Serial.printf("🔋 Voltaje: %.2f V\n", measuredV);
    voltageFeed->save(measuredV);
    lastPublish = now;
  }

  delay(10);
}

// Mover eje vertical
void moveVertical(int delta) {
  stepperY.moveTo(stepperY.currentPosition() + delta);
}

// Mover eje horizontal
void moveHorizontal(int delta) {
  stepperX.moveTo(stepperX.currentPosition() + delta);
}
