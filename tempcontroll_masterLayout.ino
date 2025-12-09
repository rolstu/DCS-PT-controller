#include <ModbusMaster.h>

// ---------- UART2 Pin Definitions ----------
#define RXD2 16     // ESP32 UART2 RX
#define TXD2 17     // ESP32 UART2 TX

// ---------- 74LV4052D MUX Select Pins ----------
#define MUX_S0 18   // connected to 74LV4052 S0
#define MUX_S1 19   // connected to 74LV4052 S1
// NOTE: If you wire the 74LV4052 enable (E) pin to a GPIO, you can add control for it here.
// For now we assume E is permanently enabled (active low) per your hardware.

// ---------- 74LV4052D DE control pins ----------
#define DE1 27
#define DE2 32
#define DE3 33
#define DE4 23

// Use DE1 as Modbus direction control to SN75176B (temperature controller)
#define RS485_DE_RE_TEMP DE1
//#define RS485_DE_RE_TEMP DE2

// Also expose macros for other DEs (for your future functions)
#define RS485_DE_RE_LIGHTS DE2
#define RS485_DE_RE_DEHUM1 DE3
#define RS485_DE_RE_DEHUM2 DE4

// ---------- Modbus Configuration (UNCHANGED) ----------
#define SLAVE_ID 1
#define BAUD_RATE 9600

ModbusMaster node;

// ---------- Register Addresses (UNCHANGED) ----------
const uint16_t REG_CONTROL_TEMP = 5;    // 40006
const uint16_t REG_SP1 = 50;            // 40051

// ---------- RS485 Direction Control (TEMP) - keep as-is ----------
void preTransmission() {
  digitalWrite(RS485_DE_RE_TEMP, HIGH);  // enable TX for temp path
  delayMicroseconds(80);
}

void postTransmission() {
  delayMicroseconds(80);
  digitalWrite(RS485_DE_RE_TEMP, LOW);   // enable RX for temp path
}

// ---------- Helper: Set only one DE pin HIGH, others LOW ----------
void setOnlyDE(uint8_t dePin) {
  // Ensure all are LOW first
  digitalWrite(DE1, LOW);
  digitalWrite(DE2, LOW);
  digitalWrite(DE3, LOW);
  digitalWrite(DE4, LOW);

  // If dePin is a valid pin (>0), set it HIGH
  if (dePin == DE1 || dePin == DE2 || dePin == DE3 || dePin == DE4) {
    digitalWrite(dePin, HIGH);
  }
}

// ---------- Helper: Select MUX channel (1..4) ----------
// Channel mapping used:
// Channel 1 -> S1=0, S0=0 (Y0)  (TEMP, DE1)
// Channel 2 -> S1=0, S0=1 (Y1)  (LIGHTS, DE2)
// Channel 3 -> S1=1, S0=0 (Y2)  (DEHUM1, DE3)
// Channel 4 -> S1=1, S0=1 (Y3)  (DEHUM2, DE4)
void selectRS485Channel(uint8_t channel) {
  switch (channel) {
    case 1: // TEMP: S1=0, S0=0
      digitalWrite(MUX_S1, LOW);
      digitalWrite(MUX_S0, LOW);
      setOnlyDE(DE1);
      break;

    case 2: // LIGHTS: S1=0, S0=1
      digitalWrite(MUX_S1, LOW);
      digitalWrite(MUX_S0, HIGH);
      setOnlyDE(DE2);
      break;

    case 3: // DEHUM1: S1=1, S0=0
      digitalWrite(MUX_S1, HIGH);
      digitalWrite(MUX_S0, LOW);
      setOnlyDE(DE3);
      break;

    case 4: // DEHUM2: S1=1, S0=1
      digitalWrite(MUX_S1, HIGH);
      digitalWrite(MUX_S0, HIGH);
      setOnlyDE(DE4);
      break;

    default:
      // invalid channel: default to safe state (all DE low)
      digitalWrite(MUX_S1, LOW);
      digitalWrite(MUX_S0, LOW);
      setOnlyDE(0); // clears all DEs
      break;
  }

  // small settle to allow mux & driver lines to stabilise
  delayMicroseconds(50);
}

// ---------- Convenience wrappers ----------
void selectTempChannel()   { selectRS485Channel(1); }
void selectLightsChannel() { selectRS485Channel(2); }
void selectDehum1Channel() { selectRS485Channel(3); }
void selectDehum2Channel() { selectRS485Channel(4); }

// ---------- Placeholders for LIGHTS / DEHUM logic ----------
// You asked to keep these ready. Implement them later as needed.
// At minimum they demonstrate how to switch the mux and DE before communicating.
void lights_sendCommand(/*parameters as needed*/) {
  // Example usage:
  // selectLightsChannel();
  // Serial2.write(...); // send your bytes for lights (non-Modbus or custom)
  // Make sure to control DE (setOnlyDE(DE2)) or use a pre/post mechanism similar to Modbus.
}

void dehum1_sendCommand(/*parameters as needed*/) {
  // selectDehum1Channel();
  // ... send commands specific to DEHUM1
}

void dehum2_sendCommand(/*parameters as needed*/) {
  // selectDehum2Channel();
  // ... send commands specific to DEHUM2
}

// ---------- Read Temperature (UNCHANGED) ----------
float readTemperature() {
  // IMPORTANT: Temperature logic uses Modbus and expects DE1 to be toggled
  // by preTransmission/postTransmission callbacks. Keep Modbus channel selected.
  selectTempChannel(); // ensure mux + DE are set for temp path before Modbus read

  uint8_t result = node.readHoldingRegisters(REG_CONTROL_TEMP, 1);

  if (result == node.ku8MBSuccess) {
    uint16_t raw = node.getResponseBuffer(0);
    return raw / 10.0;
  }
  return -999.0;
}

// ---------- Set SP1 Temperature (UNCHANGED) ----------
bool setTemperature(float tempC) {
  // ensure mux is on temp channel
  selectTempChannel();

  uint16_t regValue = (uint16_t)(tempC * 10);
  uint8_t result = node.writeSingleRegister(REG_SP1, regValue);

  if (result == node.ku8MBSuccess) {
    Serial.printf("✅ SP1 set to %.1f °C\n", tempC);

    delay(300);
    result = node.readHoldingRegisters(REG_SP1, 1);
    if (result == node.ku8MBSuccess) {
      float readBack = node.getResponseBuffer(0) / 10.0;
      Serial.printf("🔁 Verified: %.1f °C\n", readBack);
    }
    return true;
  } else {
    Serial.printf("❌ Failed, Error: 0x%02X\n", result);
    return false;
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // UART2 for Modbus
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);

  // Setup multiplexer select pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);

  // Setup multiplexer control pins initial state (channel 1 safe default)
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);

  // Setup DE pins
  pinMode(DE1, OUTPUT);
  pinMode(DE2, OUTPUT);
  pinMode(DE3, OUTPUT);
  pinMode(DE4, OUTPUT);

  // Start with all DEs LOW (RX/listen)
  digitalWrite(DE1, LOW);
  digitalWrite(DE2, LOW);
  digitalWrite(DE3, LOW);
  digitalWrite(DE4, LOW);

  // Setup Modbus node (unchanged)
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("\n===== DCS-PT Modbus Controller Ready =====");

  // Read current temp once on startup
  float temp = readTemperature();
  if (temp != -999.0) {
    Serial.printf("Current Temp: %.1f °C\n", temp);
  } else {
    Serial.println("❌ Failed to read temperature on startup");
  }
}

// ---------- Loop ----------
void loop() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (isPrintable(c) || c == '.' || c == ' ')
      input += c;

    if (c == '\r' || c == '\n') {
      input.trim();

      if (input.length() > 0) {
        Serial.print("Received command: '");
        Serial.print(input);
        Serial.println("'");

        input.toUpperCase();

        if (input.startsWith("S ")) {
          float temp = input.substring(2).toFloat();
          if (temp < 9.0 || temp > 25.0) {
            Serial.println("❌ Out of range! Use 9.0 - 25.0 °C");
          } else {
            Serial.printf("→ Setting SP1 to %.1f°C...\n", temp);
            setTemperature(temp);
          }

        } else if (input == "T") {
          float temp = readTemperature();
          if (temp != -999.0)
            Serial.printf("Current Temp: %.1f °C\n", temp);
          else
            Serial.println("❌ Failed to read");

        } else {
          Serial.println("❌ Unknown command. Use:\n  S XX.X\n  T");
        }
      }

      input = "";  // reset buffer
    }
  }

  delay(10);
}
