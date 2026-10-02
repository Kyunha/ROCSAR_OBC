#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ============================================================================
// CONFIGURAÇÃO DE HARDWARE E CINEMÁTICA
// ============================================================================
#define SERVO_BAUD          115200
#define SERVO_BROADCAST_ID  0xFE
#define BNO_SDA_PIN         16
#define BNO_SCL_PIN         17
#define HEATER_1_PIN        4
#define HEATER_2_PIN        5
#define CONTROL_LOOP_MS     20      // 50 Hz (1000ms / 20ms)

#define GEAR_RATIO          5.0f    // Gearbox 5:1
#define TICKS_PER_DEGREE    (4096.0f / 360.0f) // Encoder 12-bit
#define IMU_ALPHA           0.15f   // Filtro EMA para a bússola

// Estritura de telemetria de cada servo
struct ServoTelemetry {
  uint16_t currentTick;
  int16_t  currentSpeed;
  int16_t  currentLoad;
  float    voltageV;
  uint8_t  temperatureC;
  bool     online;
};

// Estrutura para cada eixo de antena
struct AntennaAxis {
  uint8_t id;
  uint16_t centerTick;
  float mountOffsetDeg;
  float dirMultiplier;
  bool manualMode;
  uint16_t manualTick;
  uint16_t lastSentTick;
  ServoTelemetry telemetry;

  void init(uint8_t servoId, uint16_t center, float mountOffset, float dir) {
    id = servoId;
    centerTick = center;
    mountOffsetDeg = mountOffset;
    dirMultiplier = dir;
    manualMode = false;
    manualTick = center;
    lastSentTick = 0xFFFF;
  }

  uint16_t calculateTargetTick(float gondolaHeading, float targetHeading) {
    if (manualMode) return manualTick;

    float antennaBaseWorld = fmodf(gondolaHeading + mountOffsetDeg, 360.0f);
    if (antennaBaseWorld < 0.0f) antennaBaseWorld += 360.0f;

    float rad = (targetHeading - antennaBaseWorld) * DEG_TO_RAD;
    float antennaRelativeAngle = atan2f(sinf(rad), cosf(rad)) * RAD_TO_DEG;

    float servoAngleOffset = antennaRelativeAngle * GEAR_RATIO * dirMultiplier;
    int32_t targetTick = (int32_t)centerTick + (int32_t)(servoAngleOffset * TICKS_PER_DEGREE);

    return (uint16_t)constrain(targetTick, 0, 4095);
  }
};

// ============================================================================
// VARIÁVEIS GLOBAIS
// ============================================================================
const int NUM_ANTENNAS = 2;
AntennaAxis antennas[NUM_ANTENNAS];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

float filteredGondolaHeading = 0.0f;
float globalTargetHeading = 0.0f;
unsigned long lastLoopTime = 0;
String rxBuffer = "";
bool imuOnline = false;

// ============================================================================
// FUNÇÕES AUXILIARES DE UART DOS SERVOS
// ============================================================================
void sendServoPosition(uint8_t id, uint16_t position) {
  position = constrain(position, 0, 4095);
  uint8_t pkt[13] = {
    0xFF, 0xFF, id, 0x09, 0x03, 42,
    (uint8_t)(position & 0xFF), (uint8_t)((position >> 8) & 0xFF),
    0x00, 0x00, 0x00, 0x00, 0
  };
  uint16_t sum = 0;
  for (int i = 2; i < 12; i++) sum += pkt[i];
  pkt[12] = (uint8_t)(~sum & 0xFF);

  Serial1.write(pkt, sizeof(pkt));
  Serial1.flush();
}

bool readServoTelemetry(uint8_t id, ServoTelemetry &out) {
  while (Serial1.available() > 0) Serial1.read();

  uint8_t txPkt[8] = {0xFF, 0xFF, id, 0x04, 0x02, 0x38, 0x08, 0};
  uint16_t sum = 0;
  for (int i = 2; i < 7; i++) sum += txPkt[i];
  txPkt[7] = (uint8_t)(~sum & 0xFF);

  Serial1.write(txPkt, sizeof(txPkt));
  Serial1.flush();

  unsigned long echoStart = micros();
  int echoBytesToDrain = sizeof(txPkt);
  while (echoBytesToDrain > 0 && (micros() - echoStart < 2000)) {
    if (Serial1.available() > 0) { Serial1.read(); echoBytesToDrain--; }
  }

  uint8_t rxPkt[14];
  int rxIndex = 0;
  unsigned long readStart = millis();
  while ((millis() - readStart < 5) && rxIndex < 14) {
    if (Serial1.available() > 0) rxPkt[rxIndex++] = (uint8_t)Serial1.read();
  }

  if (rxIndex < 14 || rxPkt[0] != 0xFF || rxPkt[1] != 0xFF || rxPkt[2] != id) {
    out.online = false;
    return false;
  }

  out.currentTick  = (uint16_t)(rxPkt[5] | (rxPkt[6] << 8));
  out.currentSpeed = (int16_t) (rxPkt[7] | (rxPkt[8] << 8));
  out.currentLoad  = (int16_t) (rxPkt[9] | (rxPkt[10] << 8));
  out.voltageV     = (float)rxPkt[11] / 10.0f;
  out.temperatureC = rxPkt[12];
  out.online       = true;
  return true;
}

// ============================================================================
// PROCESSADOR DE COMANDOS ASCII (OBC -> PICO)
// ============================================================================
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  int spaceIndex = cmd.indexOf(' ');
  String action = (spaceIndex == -1) ? cmd : cmd.substring(0, spaceIndex);
  String args = (spaceIndex == -1) ? "" : cmd.substring(spaceIndex + 1);

  if (action == "TARGET") {
    globalTargetHeading = args.toFloat();
    for (auto &a : antennas) a.manualMode = false;
    Serial.println("OK: TARGET_SET");
  } 
  else if (action == "HEAT1") {
    bool enable = (args == "1" || args == "ON");
    digitalWrite(HEATER_1_PIN, enable ? HIGH : LOW);
    Serial.println("OK: HEAT1");
  } 
  else if (action == "HEAT2") {
    bool enable = (args == "1" || args == "ON");
    digitalWrite(HEATER_2_PIN, enable ? HIGH : LOW);
    Serial.println("OK: HEAT2");
  } 
  else if (action == "STATUS") {
    // Formato de resposta em linha compacta ASCII para o servidor Python
    Serial.printf("TELEM,%.2f,%.2f,%d,%d,%.1f,%d,%.1f,%d\n",
                  filteredGondolaHeading, globalTargetHeading, imuOnline ? 1 : 0,
                  antennas[0].telemetry.currentTick, antennas[0].telemetry.voltageV, antennas[0].telemetry.temperatureC,
                  antennas[1].telemetry.voltageV, antennas[1].telemetry.temperatureC);
  }
}

// ============================================================================
// SETUP & LOOP PRINCIPAL
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(HEATER_1_PIN, OUTPUT);
  pinMode(HEATER_2_PIN, OUTPUT);
  digitalWrite(HEATER_1_PIN, LOW);
  digitalWrite(HEATER_2_PIN, LOW);

  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(SERVO_BAUD);

  Wire.setSDA(BNO_SDA_PIN);
  Wire.setSCL(BNO_SCL_PIN);
  Wire.begin();

  if (bno.begin()) {
    bno.setExtCrystalUse(true);
    imuOnline = true;
  }

  antennas[0].init(1, 2048, 270.0f, -1.0f);
  antennas[1].init(2, 2048, 270.0f, -1.0f);

  lastLoopTime = millis();
}

void loop() {
  // 1. Processar Comandos ASCII recebidos do OBC
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuffer.length() > 0) {
        processCommand(rxBuffer);
        rxBuffer = "";
      }
    } else {
      rxBuffer += c;
    }
  }

  // 2. Loop Determinístico de 50 Hz
  unsigned long now = millis();
  if (now - lastLoopTime >= CONTROL_LOOP_MS) {
    lastLoopTime = now;

    // Leitura da Bússola IMU
    if (imuOnline) {
      sensors_event_t event;
      bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
      float rawHeading = event.orientation.x;
      
      float rad = (rawHeading - filteredGondolaHeading) * DEG_TO_RAD;
      float delta = atan2f(sinf(rad), cosf(rad)) * RAD_TO_DEG;
      filteredGondolaHeading += delta * IMU_ALPHA;
      if (filteredGondolaHeading < 0.0f) filteredGondolaHeading += 360.0f;
      if (filteredGondolaHeading >= 360.0f) filteredGondolaHeading -= 360.0f;
    }

    // Atuação nos Servos
    for (auto &antenna : antennas) {
      uint16_t targetTick = antenna.calculateTargetTick(filteredGondolaHeading, globalTargetHeading);
      if (abs((int)targetTick - (int)antenna.lastSentTick) > 2) {
        sendServoPosition(antenna.id, targetTick);
        antenna.lastSentTick = targetTick;
      }
    }

    // Polling alternado de telemetria dos servos
    static uint8_t pollIndex = 0;
    readServoTelemetry(antennas[pollIndex].id, antennas[pollIndex].telemetry);
    pollIndex = (pollIndex + 1) % NUM_ANTENNAS;
  }
}
