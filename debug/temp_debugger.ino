#include <ModbusMaster.h>

#define RXD2 16
#define TXD2 17

#define MUX_S0 18
#define MUX_S1 19

#define DE1 27
#define DE2 32
#define DE3 33
#define DE4 23

#define RS485_DE_RE_TEMP DE1
#define RS485_DE_RE_LIGHTS DE2
#define RS485_DE_RE_DEHUM1 DE3
#define RS485_DE_RE_DEHUM2 DE4

#define SLAVE_ID 1
#define BAUD_RATE 9600

ModbusMaster node;

const uint16_t REG_CONTROL_TEMP = 5;
const uint16_t REG_SP1 = 50;

// --- RS485 direction control ---
void preTransmission() {
  digitalWrite(RS485_DE_RE_TEMP, HIGH);
  delay(3); // 3 ms TX enable
}

void postTransmission() {
  delay(3);
  digitalWrite(RS485_DE_RE_TEMP, LOW);
}

// --- Set only one DE pin HIGH ---
void setOnlyDE(uint8_t dePin) {
  digitalWrite(DE1, LOW);
  digitalWrite(DE2, LOW);
  digitalWrite(DE3, LOW);
  digitalWrite(DE4, LOW);
  if (dePin == DE1 || dePin == DE2 || dePin == DE3 || dePin == DE4)
    digitalWrite(dePin, HIGH);
}

// --- Select MUX channel ---
void selectRS485Channel(uint8_t channel) {
  switch(channel) {
    case 1: digitalWrite(MUX_S1, LOW);  digitalWrite(MUX_S0, LOW);  setOnlyDE(DE1); break;
    case 2: digitalWrite(MUX_S1, LOW);  digitalWrite(MUX_S0, HIGH); setOnlyDE(DE2); break;
    case 3: digitalWrite(MUX_S1, HIGH); digitalWrite(MUX_S0, LOW);  setOnlyDE(DE3); break;
    case 4: digitalWrite(MUX_S1, HIGH); digitalWrite(MUX_S0, HIGH); setOnlyDE(DE4); break;
    default: digitalWrite(MUX_S1, LOW); digitalWrite(MUX_S0, LOW); setOnlyDE(0); break;
  }
  delay(3); // 3 ms settle
}

void selectTempChannel() { selectRS485Channel(1); }

// --- Low-level UART debug ---
size_t uart2WriteDebug(uint8_t *data, size_t len) {
  Serial.print("TX bytes: ");
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();
  return Serial2.write(data, len);
}

int uart2ReadDebug() {
  if (Serial2.available()) {
    int c = Serial2.read();
    Serial.printf("RX byte: %02X\n", c & 0xFF);
    return c;
  }
  return -1;
}

// --- Read Temperature with debug ---
float readTemperature() {
  selectTempChannel();
  uint8_t result;
  for (uint8_t attempt=1; attempt<=2; attempt++) {
    result = node.readHoldingRegisters(REG_CONTROL_TEMP, 1);
    Serial.printf("Modbus read result: 0x%02X\n", result);
    if (result == node.ku8MBSuccess) {
      uint16_t raw = node.getResponseBuffer(0);
      Serial.printf("Raw temp register value: %d\n", raw);
      return raw / 10.0;
    }
    delay(100);
  }
  return -999.0;
}

// --- Set SP1 Temperature ---
bool setTemperature(float tempC) {
  selectTempChannel();
  uint16_t regValue = (uint16_t)(tempC * 10);
  uint8_t result = node.writeSingleRegister(REG_SP1, regValue);
  Serial.printf("Modbus write result: 0x%02X\n", result);
  if (result == node.ku8MBSuccess) return true;
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);

  pinMode(DE1, OUTPUT);
  pinMode(DE2, OUTPUT);
  pinMode(DE3, OUTPUT);
  pinMode(DE4, OUTPUT);
  digitalWrite(DE1, LOW);
  digitalWrite(DE2, LOW);
  digitalWrite(DE3, LOW);
  digitalWrite(DE4, LOW);

  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("Debug Modbus Controller Ready");
}

void loop() {
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (isPrintable(c) || c == '.' || c == ' ')
      input += c;

    if (c == '\r' || c == '\n') {
      input.trim();
      if (input.length() > 0) {
        Serial.print("Received command: ");
        Serial.println(input);
        input.toUpperCase();

        if (input.startsWith("S ")) {
          float temp = input.substring(2).toFloat();
          setTemperature(temp);
        } else if (input == "T") {
          float temp = readTemperature();
          if (temp != -999.0) Serial.printf("Current Temp: %.1f\n", temp);
          else Serial.println("Failed to read");
        }
      }
      input = "";
    }
  }

  // --- UART2 raw receive logging ---
  while (Serial2.available()) {
    int c = uart2ReadDebug();
    (void)c; // just print for debug
  }

  delay(10);
}
