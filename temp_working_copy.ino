#include <ModbusMaster.h>

// ---------- MAX485 Pin Definitions ----------
//#define RS485_DE_RE 27
#define RXD2 16
#define TXD2 17
#define MUX_S0 18
#define MUX_S1 19
#define DE1 27
#define DE2 32
#define DE3 33
#define DE4 23
#define RS485_DE_RE DE1

// ---------- Modbus Configuration ----------
#define SLAVE_ID 1
#define BAUD_RATE 9600

int activeChannel = 1;

ModbusMaster node;

// ---------- Register Addresses ----------
const uint16_t REG_CONTROL_TEMP = 5;   // 40006 - Current temperature
const uint16_t REG_SP1 = 50;           // 40051 - Set Point 1
const uint16_t REG_CS = 52;            // 40053 - Compressor Selection
const uint16_t REG_HS = 55;            // 40056 - High Limit
const uint16_t REG_LS = 56;            // 40057 - Low Limit

// ---------- RS485 Control ----------
void preTransmission() {
  digitalWrite(DE1, HIGH);
  delayMicroseconds(80);
  //delay(10);
}

void postTransmission() {
  //Serial.println(activeChannel);
  delayMicroseconds(80);
  //delay(10);
  digitalWrite(DE1,LOW);
}

// ---------- Read Temperature ----------
float readTemperature() {
  activeChannel =  1;
  uint8_t result = node.readHoldingRegisters(REG_CONTROL_TEMP, 1);
  
  if (result == node.ku8MBSuccess) {
    uint16_t raw = node.getResponseBuffer(0);
    return raw / 10.0;
  }
  return -999.0;
}

// ---------- Set SP1 Temperature ----------
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

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
  
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);

  pinMode(DE1, OUTPUT);
  pinMode(DE2, OUTPUT);
  pinMode(DE3, OUTPUT);
  pinMode(DE4, OUTPUT);

  // Start with all DEs LOW (RX/listen)
  digitalWrite(DE1, LOW);
  digitalWrite(DE2, LOW);
  digitalWrite(DE3, LOW);
  digitalWrite(DE4, LOW);
 
  // Setup multiplexer control pins initial state (channel 1 safe default)
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║  DCS-PT Simple Control v1.0      ║");
  Serial.println("╚═══════════════════════════════════╝");
  Serial.println("\nCommands:");
  Serial.println("  S XX.X - Set temperature (e.g., S 15.5)");
  Serial.println("  T      - Read current temperature");
  //Serial.println("  CHECK  - Verify settings");
  //Serial.println("\nExpected Settings:");
  //Serial.println("  CS: 0 (Comp 1 Only)");
  //Serial.println("  P4: 1.0°C (Differential)");
  //Serial.println("  HS: 26.0°C | LS: 8.0°C");
  //Serial.println("  Valid SP1 range: 9.0°C - 25.0°C");
  Serial.println("\n✅ Ready!");
  
  // Show initial temperature
  float temp = readTemperature();
  if (temp != -999.0) {
    Serial.print("\nCurrent: ");
    Serial.print(temp, 1);
    Serial.println(" °C");
  }
}

// ---------- Loop ----------
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();
    
    if (input.startsWith("S ")) {
      float temp = input.substring(2).toFloat();
      
      // Validate range: LS+1 to HS-1 = 9.0 to 25.0
      if (temp < 8.0 || temp > 26.0) {
        Serial.println("❌ Out of range! Use 9.0 - 25.0 °C");
        return;
      }
      
      Serial.print("\n→ Setting SP1 to ");
      Serial.print(temp, 1);
      Serial.println("°C...");
      setTemperature(temp);
      
    } else if (input == "T") {
      float temp = readTemperature();
      if (temp != -999.0) {
        Serial.print("\nCurrent: ");
        Serial.print(temp, 1);
        Serial.println(" °C");
      } else {
        Serial.println("\n❌ Failed to read temperature");
      }
      
    } else if (input == "CHECK") {
      Serial.println("\n--- Checking Controller Settings ---");
      
      // Check CS
      uint8_t result = node.readHoldingRegisters(REG_CS, 1);
      if (result == node.ku8MBSuccess) {
        uint16_t cs = node.getResponseBuffer(0);
        Serial.print("CS: ");
        Serial.print(cs);
        Serial.print(" (");
        switch(cs) {
          case 0: Serial.print("C1-Comp1 Only"); break;
          case 1: Serial.print("C2-Comp2 Only - ⚠️ SP1 BLOCKED!"); break;
          case 2: Serial.print("btH-Both"); break;
          case 3: Serial.print("Stb-Standby"); break;
        }
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
      }
      
      delay(100);
      
      // Check LS
      result = node.readHoldingRegisters(REG_LS, 1);
      if (result == node.ku8MBSuccess) {
        float ls = node.getResponseBuffer(0) / 10.0;
        Serial.print("LS (Low Limit):  ");
        Serial.print(ls, 1);
        Serial.println(" °C");
      }
      
      delay(100);
      
      // Check current SP1
      result = node.readHoldingRegisters(REG_SP1, 1);
      if (result == node.ku8MBSuccess) {
        float sp1 = node.getResponseBuffer(0) / 10.0;
        Serial.print("SP1 (Current):   ");
        Serial.print(sp1, 1);
        Serial.println(" °C");
      }
      
    } else if (input.length() > 0) {
      Serial.println("❌ Unknown. Use: S XX.X, T, or CHECK");
    }
  }
  
  delay(100);
}
