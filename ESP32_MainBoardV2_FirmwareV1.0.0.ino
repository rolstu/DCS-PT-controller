/*
  Merged firmware:
  - ModbusMaster-based DCS-PT controller interface (S, T, CHECK)
  - PCF8575 relay expander (I2C) with interrupt and relay commands
*/

#include <Wire.h>
#include <ModbusMaster.h>

// -----------------------------
// Modbus / RS485 Configuration
// -----------------------------
#define RS485_DE_RE 27   // MAX485 / SN75176 DE/RE pin DE1
#define RXD2 16
#define TXD2 17
#define MUX_S0 18
#define MUX_S1 19

#define SLAVE_ID 1
#define BAUD_RATE 9600

ModbusMaster node;

// Modbus registers
const uint16_t REG_CONTROL_TEMP = 5;   // 40006 - Current temperature
const uint16_t REG_SP1 = 50;           // 40051 - Set Point 1
const uint16_t REG_CS = 52;            // 40053 - Compressor Selection
const uint16_t REG_HS = 55;            // 40056 - High Limit
const uint16_t REG_LS = 56;            // 40057 - Low Limit

// -----------------------------
// PCF8575 (I2C) Configuration
// -----------------------------
#define PCF8575_ADDR   0x20
#define INT_RLY_EXP    39

volatile bool pcfInterrupt = false;
uint16_t pcfState = 0xFFFF; // HIGH = OFF (inverted relays) — default all off

// -----------------------------
// Forward declarations
// -----------------------------
void preTransmission();
void postTransmission();
float readTemperature();
bool setTemperature(float tempC);
void writePCF(uint16_t data);
uint16_t readPCF();
void IRAM_ATTR pcfISR();
bool relayIsOn(uint8_t r);
void setRelay(uint8_t r, bool on);

// -----------------------------
// RS485 Direction Control
// -----------------------------
void preTransmission() {
  // Enable TX
  digitalWrite(RS485_DE_RE, HIGH);
  delayMicroseconds(80);
}

void postTransmission() {
  // Back to RX
  delayMicroseconds(80);
  digitalWrite(RS485_DE_RE, LOW);
}

// -----------------------------
// Modbus: Read Temperature
// -----------------------------
float readTemperature() {
  uint8_t result = node.readHoldingRegisters(REG_CONTROL_TEMP, 1);

  if (result == node.ku8MBSuccess) {
    uint16_t raw = node.getResponseBuffer(0);
    return raw / 10.0;
  }
  return -999.0;
}

// -----------------------------
// Modbus: Set SP1 Temperature
// -----------------------------
bool setTemperature(float tempC) {
  // First check CS value
  uint8_t result = node.readHoldingRegisters(REG_CS, 1);
  if (result == node.ku8MBSuccess) {
    uint16_t cs = node.getResponseBuffer(0);
    if (cs == 1) {
      Serial.println("❌ Error: CS=1 (Comp 2 only)");
      Serial.println("   Cannot modify SP1 when CS=1");
      Serial.println("   Change CS to 0, 2, or 3 on controller");
      return false;
    }
  } else {
    Serial.print("⚠️ Could not read CS before writing (Modbus code 0x");
    Serial.print(result, HEX);
    Serial.println("). Aborting write.");
    // proceed? safer to abort
    return false;
  }

  delay(100);

  // Write SP1
  uint16_t regValue = (uint16_t)(tempC * 10);
  result = node.writeSingleRegister(REG_SP1, regValue);

  if (result == node.ku8MBSuccess) {
    Serial.print("✅ SP1 set to ");
    Serial.print(tempC, 1);
    Serial.println(" °C");

    // Verify
    delay(500);
    result = node.readHoldingRegisters(REG_SP1, 1);
    if (result == node.ku8MBSuccess) {
      float readBack = node.getResponseBuffer(0) / 10.0;
      Serial.print("🔁 Verified: ");
      Serial.print(readBack, 1);
      Serial.println(" °C");
    } else {
      Serial.print("⚠️ Verification read failed (0x");
      Serial.print(result, HEX);
      Serial.println(")");
    }
    return true;
  } else {
    Serial.print("❌ Failed, Error: 0x");
    Serial.println(result, HEX);

    if (result == 0x03) {
      Serial.println("\n💡 Troubleshooting Error 0x3:");
      Serial.println("   1. Check CS value (type: CHECK)");
      Serial.println("   2. Verify HS/LS limits (type: CHECK)");
      Serial.println("   3. Temperature must be 9.0-25.0°C");
    }
    return false;
  }
}

// -----------------------------
// PCF8575 Helpers
// -----------------------------
void writePCF(uint16_t data) {
  // Writes low byte first, then high byte
  Wire.beginTransmission(PCF8575_ADDR);
  Wire.write(lowByte(data));
  Wire.write(highByte(data));
  Wire.endTransmission();
}

uint16_t readPCF() {
  Wire.requestFrom(PCF8575_ADDR, (uint8_t)2);
  // Wait until two bytes available (small blocking)
  unsigned long start = millis();
  while (Wire.available() < 2 && (millis() - start) < 50) {
    // wait up to 50 ms
  }
  uint8_t low = 0xFF;
  uint8_t high = 0xFF;
  if (Wire.available()) low = Wire.read();
  if (Wire.available()) high = Wire.read();
  return (uint16_t)high << 8 | low;
}

void IRAM_ATTR pcfISR() {
  pcfInterrupt = true;
}

// Relay helpers: relays numbered 1..10 (as in previous sketch)
bool relayIsOn(uint8_t r) {
  if (r < 1 || r > 16) return false; // PCF8575 has 16 bits; user used 1..10
  return !(pcfState & (1 << (r - 1))); // LOW = ON (inverted)
}

void setRelay(uint8_t r, bool on) {
  if (r < 1 || r > 16) return;

  if (on)
    pcfState &= ~(1 << (r - 1)); // LOW = ON
  else
    pcfState |= (1 << (r - 1));  // HIGH = OFF

  writePCF(pcfState);

  Serial.print("Relay ");
  if (r < 10) Serial.print("0"); // match earlier formatting for R01
  Serial.print(r);
  Serial.println(on ? " ON" : " OFF");
}

// -----------------------------
// Setup
// -----------------------------
void setup() {
  // Serial console
  Serial.begin(115200);
  delay(50);

  // I2C for PCF8575 (SDA=21, SCL=22)
  Wire.begin(21, 22);

  // PCF INT pin
  pinMode(INT_RLY_EXP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_RLY_EXP), pcfISR, FALLING);

  // Initialize PCF state and write initial value
  writePCF(pcfState);

  // RS485 / Modbus Serial2
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);

  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);

  // MUX pins configured (left for future use; keep safe default)
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);

  // Modbus node setup
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // Console banner and help
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║  DCS-PT + PCF8575 Merged v1.0    ║");
  Serial.println("╚═══════════════════════════════════╝");
  Serial.println("\nModbus Commands:");
  Serial.println("  S XX.X   - Set SP1 (e.g., S 15.5)");
  Serial.println("  T        - Read current temperature");
  Serial.println("  CHECK    - Verify CS / HS / LS / SP1");
  Serial.println("\nRelay Commands:");
  Serial.println("  R01 ON   - Turn relay 1 ON");
  Serial.println("  R01 OFF  - Turn relay 1 OFF");
  Serial.println("  R01?     - Query relay 1 state");
  Serial.println("  STATUS   - Print all relay states");
  Serial.println("  ALL OFF  - Turn all relays OFF");
  Serial.println("\n✅ Ready!");

  // Show initial temperature (best-effort)
  float temp = readTemperature();
  if (temp != -999.0) {
    Serial.print("\nCurrent: ");
    Serial.print(temp, 1);
    Serial.println(" °C");
  } else {
    Serial.println("\n(Current temperature read failed)");
  }
}

// -----------------------------
// Loop: combined command parser
// -----------------------------
void loop() {
  // Serial commands (both Modbus and Relay commands handled here)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    // ---- Relay commands & STATUS ----
    if (input == "STATUS") {
      for (int i = 1; i <= 8; i++) {
        Serial.print("R");
        if (i < 8) Serial.print("0");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(relayIsOn(i) ? "ON" : "OFF");
      }
      return;
    }

    if (input == "ALL OFF") {
      pcfState = 0xFFFF;
      writePCF(pcfState);
      Serial.println("ALL RELAYS OFF");
      return;
    }

    if (input.startsWith("R") && input.length() >= 3) {
      // Format R01 ON | R01 OFF | R01?  (Rnn)
      String idxStr = input.substring(1, 3); // two digits
      int r = idxStr.toInt();
      if (r < 1 || r > 16) {
        Serial.println("Invalid relay number (1-16)");
        return;
      }

      if (input.endsWith("?")) {
        Serial.print("R");
        if (r < 10) Serial.print("0");
        Serial.print(r);
        Serial.print(": ");
        Serial.println(relayIsOn(r) ? "ON" : "OFF");
        return;
      }

      if (input.endsWith("ON")) {
        setRelay(r, true);
        return;
      }

      if (input.endsWith("OFF")) {
        setRelay(r, false);
        return;
      }

      Serial.println("Invalid relay command. Use R01 ON | R01 OFF | R01?");
      return;
    }

    // ---- Modbus / Temperature commands ----
    if (input.startsWith("S ")) {
      // Set SP
      float temp = input.substring(2).toFloat();

      // Validate range: LS+1 to HS-1 = roughly 9.0 to 25.0 (keep same checks as before)
      if (temp < 8.0 || temp > 26.0) {
        Serial.println("❌ Out of range! Use 9.0 - 25.0 °C");
        return;
      }

      Serial.print("\n→ Setting SP1 to ");
      Serial.print(temp, 1);
      Serial.println("°C...");
      setTemperature(temp);
      return;
    }

    if (input == "T") {
      float temp = readTemperature();
      if (temp != -999.0) {
        Serial.print("\nCurrent: ");
        Serial.print(temp, 1);
        Serial.println(" °C");
      } else {
        Serial.println("\n❌ Failed to read temperature");
      }
      return;
    }

    if (input == "CHECK") {
      Serial.println("\n--- Checking Controller Settings ---");

      // Check CS
      uint8_t result = node.readHoldingRegisters(REG_CS, 1);
      if (result == node.ku8MBSuccess) {
        uint16_t cs = node.getResponseBuffer(0);
        Serial.print("CS: ");
        Serial.print(cs);
        Serial.print(" (");
        switch (cs) {
          case 0: Serial.print("C1-Comp1 Only"); break;
          case 1: Serial.print("C2-Comp2 Only - ⚠️ SP1 BLOCKED!"); break;
          case 2: Serial.print("btH-Both"); break;
          case 3: Serial.print("Stb-Standby"); break;
          default: Serial.print("Unknown"); break;
        }
        Serial.println(")");
      } else {
        Serial.print("CS read failed (0x");
        Serial.print(result, HEX);
        Serial.println(")");
      }

      delay(100);

      // Check HS
      result = node.readHoldingRegisters(REG_HS, 1);
      if (result == node.ku8MBSuccess) {
        float hs = node.getResponseBuffer(0) / 10.0;
        Serial.print("HS (High Limit): ");
        Serial.print(hs, 1);
        Serial.println(" °C");
      } else {
        Serial.print("HS read failed (0x");
        Serial.print(result, HEX);
        Serial.println(")");
      }

      delay(100);

      // Check LS
      result = node.readHoldingRegisters(REG_LS, 1);
      if (result == node.ku8MBSuccess) {
        float ls = node.getResponseBuffer(0) / 10.0;
        Serial.print("LS (Low Limit):  ");
        Serial.print(ls, 1);
        Serial.println(" °C");
      } else {
        Serial.print("LS read failed (0x");
        Serial.print(result, HEX);
        Serial.println(")");
      }

      delay(100);

      // Check current SP1
      result = node.readHoldingRegisters(REG_SP1, 1);
      if (result == node.ku8MBSuccess) {
        float sp1 = node.getResponseBuffer(0) / 10.0;
        Serial.print("SP1 (Current):   ");
        Serial.print(sp1, 1);
        Serial.println(" °C");
      } else {
        Serial.print("SP1 read failed (0x");
        Serial.print(result, HEX);
        Serial.println(")");
      }

      return;
    }

    // Unknown command
    if (input.length() > 0) {
      Serial.println("❌ Unknown. Use: S XX.X, T, CHECK, R01 ON/OFF/? , STATUS, or ALL OFF");
    }
  } // end Serial.available()

  // ----------------------------
  // PCF8575 interrupt handling
  // ----------------------------
  if (pcfInterrupt) {
    pcfInterrupt = false;
    pcfState = readPCF();
    Serial.println("PCF8575 INT sync");
  }

  delay(100);
}
