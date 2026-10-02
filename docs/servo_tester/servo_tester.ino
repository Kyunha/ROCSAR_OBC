#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ============================================================================
// HARDWARE & KINEMATIC CONFIGURATION
// ============================================================================
#define SERVO_BAUD          115200  // ST3215 UART Baud Rate
#define SERVO_BROADCAST_ID  0xFE     // 254: Universal Broadcast ID for all ST3215 servos
#define BNO_SDA_PIN         16       // RP2040 I2C SDA
#define BNO_SCL_PIN         17       // RP2040 I2C SCL
#define HEATER_1_PIN        4        // Servo 1 Heating Module GPIO
#define HEATER_2_PIN        5        // Servo 2 Heating Module GPIO
#define CONTROL_LOOP_MS     20       // 50 Hz Update Rate

// 5:1 Gearbox: 1 degree of antenna rotation requires 5 degrees of servo movement
#define GEAR_RATIO          5.0f     
#define TICKS_PER_DEGREE    (4096.0f / 360.0f) // ST3215 12-bit Encoder
#define IMU_ALPHA           0.15f    // EMA Filter weight

// ============================================================================
// ST3215 TELEMETRY DATA STRUCTURE
// ============================================================================
struct ServoTelemetry {
  uint16_t currentTick;
  int16_t  currentSpeed;
  int16_t  currentLoad;
  float    voltageV;
  uint8_t  temperatureC;
  bool     online;
  unsigned long lastReadMs;
};

// ============================================================================
// MATH HELPERS
// ============================================================================
float wrap360(float deg) {
  deg = fmodf(deg, 360.0f);
  return (deg < 0.0f) ? deg + 360.0f : deg;
}

float wrap180(float deg) {
  float rad = deg * DEG_TO_RAD;
  return atan2f(sinf(rad), cosf(rad)) * RAD_TO_DEG;
}

// ============================================================================
// ANTENNA AXIS CLASS
// ============================================================================
struct AntennaAxis {
  uint8_t id;
  uint16_t centerTick;     // Servo tick corresponding to physical 0° alignment
  float mountOffsetDeg;    // Physical mounting angle relative to IMU forward axis
  float dirMultiplier;     // 1.0f or -1.0f (Inverts physical rotation)
  
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
    
    telemetry.currentTick = 0;
    telemetry.currentSpeed = 0;
    telemetry.currentLoad = 0;
    telemetry.voltageV = 0.0f;
    telemetry.temperatureC = 0;
    telemetry.online = false;
    telemetry.lastReadMs = 0;
  }
  uint16_t calculateTargetTick(float gondolaHeading, float targetHeading) {
    if (manualMode) return manualTick;

    // 1. Determine where this specific antenna base is pointing in world coordinates
    float antennaBaseWorld = wrap360(gondolaHeading + mountOffsetDeg);

    // 2. Compute shortest relative angle between antenna base and target (-180° to +180°)
    float antennaRelativeAngle = wrap180(targetHeading - antennaBaseWorld);

    // 3. Scale by 5:1 gearbox reduction and direction sign
    float servoAngleOffset = antennaRelativeAngle * GEAR_RATIO * dirMultiplier;

    // 4. Convert servo angle offset into encoder ticks
    int32_t tickOffset = (int32_t)(servoAngleOffset * TICKS_PER_DEGREE);
    int32_t targetTick = (int32_t)centerTick + tickOffset;

    // 5. Constrain within physical 12-bit encoder limits
    return (uint16_t)constrain(targetTick, 0, 4095);
  }
};

// ============================================================================
// GLOBAL STATE
// ============================================================================
const int NUM_ANTENNAS = 2;
AntennaAxis antennas[NUM_ANTENNAS];

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

float filteredGondolaHeading = 0.0f;
float globalTargetHeading = 0.0f;
unsigned long lastLoopTime = 0;
String rxBuffer = "";

bool heater1State = false;
bool heater2State = false;

// ============================================================================
// ST3215 SERVO UART DRIVER (WRITE & READ)
// ============================================================================
void sendServoPosition(uint8_t id, uint16_t position) {
  position = constrain(position, 0, 4095);
  
  uint8_t pkt[13] = {
    0xFF, 0xFF, id, 0x09, 0x03, 42,
    (uint8_t)(position & 0xFF), (uint8_t)((position >> 8) & 0xFF),
    0x00, 0x00, // Goal Time = 0 (Immediate)
    0x00, 0x00, // Goal Speed = 0
    0
  };

  uint16_t sum = 0;
  for (int i = 2; i < 12; i++) sum += pkt[i];
  pkt[12] = (uint8_t)(~sum & 0xFF);

  Serial1.write(pkt, sizeof(pkt));
  Serial1.flush();
}

void sendBroadcastPosition(uint16_t position) {
  sendServoPosition(SERVO_BROADCAST_ID, position);
}

/**
 * Reads 8 telemetry bytes starting at Register 0x38 (56) from a specific servo ID.
 * Returns true if valid packet received with checksum match.
 */
bool readServoTelemetry(uint8_t id, ServoTelemetry &out) {
  if (id == SERVO_BROADCAST_ID) return false;

  // Flush any leftover stale bytes in RX FIFO
  while (Serial1.available() > 0) Serial1.read();

  // Construct READ DATA packet (INST = 0x02, Start Reg = 0x38, Read Len = 8)
  uint8_t txPkt[8] = {
    0xFF, 0xFF, id, 0x04, 0x02, 0x38, 0x08, 0
  };
  
  uint16_t sum = 0;
  for (int i = 2; i < 7; i++) sum += txPkt[i];
  txPkt[7] = (uint8_t)(~sum & 0xFF);

  // Transmit Read Request
  Serial1.write(txPkt, sizeof(txPkt));
  Serial1.flush();

  // Handle single-bus half-duplex echo (drain the 8 bytes sent)
  unsigned long echoStart = micros();
  int echoBytesToDrain = sizeof(txPkt);
  while (echoBytesToDrain > 0 && (micros() - echoStart < 2000)) {
    if (Serial1.available() > 0) {
      Serial1.read();
      echoBytesToDrain--;
    }
  }

  // Expecting 14 bytes response: [0xFF, 0xFF, ID, LEN(0x0A), ERR, POS_L, POS_H, SPD_L, SPD_H, LOAD_L, LOAD_H, VOLT, TEMP, CHK]
  uint8_t rxPkt[14];
  int rxIndex = 0;
  unsigned long readStart = millis();

  while ((millis() - readStart < 5) && rxIndex < 14) { // 5ms timeout
    if (Serial1.available() > 0) {
      rxPkt[rxIndex++] = (uint8_t)Serial1.read();
    }
  }

  if (rxIndex < 14) {
    out.online = false;
    return false; // Timeout or truncated frame
  }

  // Validate Header and ID
  if (rxPkt[0] != 0xFF || rxPkt[1] != 0xFF || rxPkt[2] != id) {
    out.online = false;
    return false;
  }

  // Verify Checksum
  uint16_t rxSum = 0;
  for (int i = 2; i < 13; i++) rxSum += rxPkt[i];
  uint8_t calculatedChk = (uint8_t)(~rxSum & 0xFF);

  if (calculatedChk != rxPkt[13]) {
    out.online = false;
    return false; // Checksum error
  }

  // Parse Telemetry Fields
  out.currentTick   = (uint16_t)(rxPkt[5] | (rxPkt[6] << 8));
  out.currentSpeed  = (int16_t) (rxPkt[7] | (rxPkt[8] << 8));
  out.currentLoad   = (int16_t) (rxPkt[9] | (rxPkt[10] << 8));
  out.voltageV      = (float)rxPkt[11] / 10.0f; // 0.1V resolution
  out.temperatureC  = rxPkt[12];                // °C
  out.online        = true;
  out.lastReadMs    = millis();

  return true;
}

// ============================================================================
// BROADCAST TEST ROUTINE
// ============================================================================
void runBroadcastSweepTest() {
  Serial.println("\n==================================================");
  Serial.println("  STARTING BROADCAST SWEEP TEST (ALL SERVOS)     ");
  Serial.println("==================================================");
  
  uint16_t centerTick = 2048;
  uint16_t leftTick   = 1800; // ~ -21.8° rotation
  uint16_t rightTick  = 2296; // ~ +21.8° rotation

  Serial.println("--> Broadcast: Moving to Center (2048)...");
  sendBroadcastPosition(centerTick);
  delay(1000);

  Serial.println("--> Broadcast: Rotating Left (1800)...");
  sendBroadcastPosition(leftTick);
  delay(1000);

  Serial.println("--> Broadcast: Rotating Right (2296)...");
  sendBroadcastPosition(rightTick);
  delay(1000);

  Serial.println("--> Broadcast: Returning to Center (2048)...");
  sendBroadcastPosition(centerTick);
  delay(1000);

  Serial.println("==================================================");
  Serial.println("  BROADCAST SWEEP TEST COMPLETE                  ");
  Serial.println("==================================================\n");
}

// ============================================================================
// SERIAL CLI COMMAND PARSER
// ============================================================================
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  int spaceIndex = cmd.indexOf(' ');
  String action = (spaceIndex == -1) ? cmd : cmd.substring(0, spaceIndex);
  String args = (spaceIndex == -1) ? "" : cmd.substring(spaceIndex + 1);

  if (action == "TARGET") {
    globalTargetHeading = wrap360(args.toFloat());
    for (auto &a : antennas) a.manualMode = false;
    Serial.printf("--> TARGET SET: %.1f°\n", globalTargetHeading);
  }
  else if (action == "JOG") {
    int id = args.substring(0, args.indexOf(' ')).toInt();
    int tick = args.substring(args.indexOf(' ') + 1).toInt();
    for (auto &a : antennas) {
      if (a.id == id) {
        a.manualTick = constrain(tick, 0, 4095);
        a.manualMode = true;
        Serial.printf("--> SERVO %d JOGGED TO TICK %d\n", id, a.manualTick);
      }
    }
  }
  else if (action == "READ") {
    int id = args.toInt();
    ServoTelemetry t;
    if (readServoTelemetry(id, t)) {
      Serial.printf("--> SERVO %d TELEMETRY: Pos: %4d | Speed: %4d | Load: %4d | Volt: %.1fV | Temp: %d°C\n",
                    id, t.currentTick, t.currentSpeed, t.currentLoad, t.voltageV, t.temperatureC);
    } else {
      Serial.printf("--> SERVO %d READ FAILED (Offline or Timeout)\n", id);
    }
  }
  else if (action == "BROADCAST") {
    int tick = args.toInt();
    uint16_t targetTick = constrain(tick, 0, 4095);
    sendBroadcastPosition(targetTick);
    Serial.printf("--> BROADCAST SENT TO ALL SERVOS (ID 0xFE): TICK %d\n", targetTick);
  }
  else if (action == "TEST" || action == "SWEEP") {
    runBroadcastSweepTest();
  }
  else if (action == "ZERO") {
    int id = args.toInt();
    for (auto &a : antennas) {
      if (a.id == id && a.manualMode) {
        a.centerTick = a.manualTick;
        Serial.printf("--> SERVO %d NEW CENTER SAVED: %d\n", id, a.centerTick);
      }
    }
  }
  else if (action == "MOUNT") {
    int id = args.substring(0, args.indexOf(' ')).toInt();
    float angle = args.substring(args.indexOf(' ') + 1).toFloat();
    for (auto &a : antennas) {
      if (a.id == id) {
        a.mountOffsetDeg = wrap360(angle);
        Serial.printf("--> SERVO %d MOUNT OFFSET: %.1f°\n", id, a.mountOffsetDeg);
      }
    }
  }
  else if (action == "DIR") {
    int id = args.substring(0, args.indexOf(' ')).toInt();
    float dir = args.substring(args.indexOf(' ') + 1).toFloat();
    for (auto &a : antennas) {
      if (a.id == id) {
        a.dirMultiplier = (dir < 0) ? -1.0f : 1.0f;
        Serial.printf("--> SERVO %d DIR MULTIPLIER: %.1f\n", id, a.dirMultiplier);
      }
    }
  }
  else if (action == "HEAT1") {
    args.trim();
    bool enable = (args == "1" || args == "ON" || args == "HIGH");
    heater1State = enable;
    digitalWrite(HEATER_1_PIN, enable ? HIGH : LOW);
    Serial.printf("--> HEATER 1 (GPIO 4): %s\n", enable ? "ON" : "OFF");
  }
  else if (action == "HEAT2") {
    args.trim();
    bool enable = (args == "1" || args == "ON" || args == "HIGH");
    heater2State = enable;
    digitalWrite(HEATER_2_PIN, enable ? HIGH : LOW);
    Serial.printf("--> HEATER 2 (GPIO 5): %s\n", enable ? "ON" : "OFF");
  }
  else if (action == "HEAT") {
    int firstSpace = args.indexOf(' ');
    int heaterNum = (firstSpace == -1) ? args.toInt() : args.substring(0, firstSpace).toInt();
    String stateStr = (firstSpace == -1) ? "" : args.substring(firstSpace + 1);
    stateStr.trim();
    bool enable = (stateStr == "1" || stateStr == "ON" || stateStr == "HIGH");

    if (heaterNum == 1) {
      heater1State = enable;
      digitalWrite(HEATER_1_PIN, enable ? HIGH : LOW);
      Serial.printf("--> HEATER 1 (GPIO 4): %s\n", enable ? "ON" : "OFF");
    } else if (heaterNum == 2) {
      heater2State = enable;
      digitalWrite(HEATER_2_PIN, enable ? HIGH : LOW);
      Serial.printf("--> HEATER 2 (GPIO 5): %s\n", enable ? "ON" : "OFF");
    }
  }
  else if (action == "STATUS") {
    Serial.printf("\n--- SYSTEM STATUS ---\n");
    Serial.printf("Gondola Heading: %.1f°\n", filteredGondolaHeading);
    Serial.printf("Target Heading:  %.1f°\n", globalTargetHeading);
    Serial.printf("Heater 1 (GPIO 4): %s\n", heater1State ? "ON" : "OFF");
    Serial.printf("Heater 2 (GPIO 5): %s\n", heater2State ? "ON" : "OFF");
    for (auto &a : antennas) {
      // Force immediate telemetry read for status display
      readServoTelemetry(a.id, a.telemetry);

      Serial.printf("Servo ID %2d | Mode: %s | Center: %4d | Mount: %5.1f° | Dir: %2.0f\n",
                    a.id, a.manualMode ? "MANUAL" : "TRACK ",
                    a.centerTick, a.mountOffsetDeg, a.dirMultiplier);
      if (a.telemetry.online) {
        Serial.printf("         --> Telemetry: Pos: %4d | Spd: %4d | Load: %4d | Volt: %4.1fV | Temp: %2d°C\n",
                      a.telemetry.currentTick, a.telemetry.currentSpeed,
                      a.telemetry.currentLoad, a.telemetry.voltageV, a.telemetry.temperatureC);
      } else {
        Serial.printf("         --> Telemetry: [SERVO OFFLINE / TIMEOUT]\n");
      }
    }
    Serial.println("---------------------\n");
  }
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Setup Heater Control GPIOs
  pinMode(HEATER_1_PIN, OUTPUT);
  pinMode(HEATER_2_PIN, OUTPUT);
  digitalWrite(HEATER_1_PIN, LOW); // Start OFF
  digitalWrite(HEATER_2_PIN, LOW); // Start OFF

  // Setup ST3215 Servo Serial Bus on RP2040 UART0 (TX=0, RX=1)
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(SERVO_BAUD);

  // Setup I2C for BNO055 IMU
  Wire.setSDA(BNO_SDA_PIN);
  Wire.setSCL(BNO_SCL_PIN);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 IMU not detected on I2C bus!");
  } else {
    bno.setExtCrystalUse(true);
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    filteredGondolaHeading = event.orientation.x;
  }

  // Initialize Servo 1 and Servo 2 Configuration
  antennas[0].init(1,  2048, 270.0f, -1.0f);
  antennas[1].init(2, 2048, 270.0f, -1.0f);

  rxBuffer.reserve(64);
  
  // Run initial broadcast test rotation on startup
  delay(1000);
  runBroadcastSweepTest();

  lastLoopTime = millis();
}

void loop() {
  // Parse Incoming USB Serial Commands
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

  // Deterministic 50 Hz Control Loop
  unsigned long now = millis();
  if (now - lastLoopTime >= CONTROL_LOOP_MS) {
    lastLoopTime = now;

    // 1. Read Raw Yaw from BNO055
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    float rawHeading = event.orientation.x;

    // 2. Smooth Heading via Circular EMA Filter
    float deltaHeading = wrap180(rawHeading - filteredGondolaHeading);
    filteredGondolaHeading = wrap360(filteredGondolaHeading + (deltaHeading * IMU_ALPHA));

    // 3. Update Kinematics and Actuate Servos
    for (auto &antenna : antennas) {
      uint16_t targetTick = antenna.calculateTargetTick(filteredGondolaHeading, globalTargetHeading);

      // Deadband filter: send commands only if target tick changes by > 2 ticks (~0.17°)
      if (abs((int)targetTick - (int)antenna.lastSentTick) > 2) {
        sendServoPosition(antenna.id, targetTick);
        antenna.lastSentTick = targetTick;
      }
    }

    // 4. Interleaved Telemetry Polling (Polls 1 servo per loop tick to preserve 50Hz timing)
    static uint8_t pollIndex = 0;
    readServoTelemetry(antennas[pollIndex].id, antennas[pollIndex].telemetry);
    pollIndex = (pollIndex + 1) % NUM_ANTENNAS;
  }
}