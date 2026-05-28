#include <RF24.h>

RF24 radio(2, 4);
static constexpr uint8_t Address[5] = {0xAB, 0xCD, 0xAB, 0xCD, 0x71};

// Период отправки пакетов (мс). Приёмник имеет fail-safe 500 мс,
// поэтому шлём чаще, чтобы не уходить в стоп при тишине от Serial.
static constexpr unsigned long SEND_INTERVAL_MS = 50;

int8_t  speed = 0;   // -128..127
uint8_t angle = 90;  // 0..180

// Буфер строки из Serial
static char lineBuf[32];
static uint8_t lineLen = 0;

static void parseLine(const char* s) {
  // Формат: "<speed>,<angle>"
  // speed: -128..127, angle: 0..180
  char* end;
  long sp = strtol(s, &end, 10);
  if (end == s || *end != ',') {
    Serial.println(F("ERR: expected 'speed,angle'"));
    return;
  }
  const char* p = end + 1;
  long an = strtol(p, &end, 10);
  if (end == p) {
    Serial.println(F("ERR: bad angle"));
    return;
  }

  if (sp < -128) sp = -128;
  if (sp >  127) sp =  127;
  if (an < 0)    an = 0;
  if (an > 180)  an = 180;

  speed = (int8_t)sp;
  angle = (uint8_t)an;

  Serial.print(F("OK speed="));
  Serial.print(speed);
  Serial.print(F(" angle="));
  Serial.println(angle);
}

static void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) parseLine(lineBuf);
      lineLen = 0;
      continue;
    }
    if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      // Переполнение — сбрасываем строку
      lineLen = 0;
      Serial.println(F("ERR: line too long"));
    }
  }
}

static void sendPacket() {
  uint8_t buffer[2];
  buffer[0] = (uint8_t)speed;  // приёмник кастует обратно к int8_t
  buffer[1] = angle;
  bool ok = radio.write(buffer, sizeof(buffer));
  if (!ok) {
    Serial.println(F("TX fail"));
  }
}

void setup() {
  Serial.begin(115200);

  radio.begin();
  radio.setChannel(101);
  radio.setPayloadSize(2);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(15, 15);
  radio.openWritingPipe(Address);
  radio.stopListening();

  Serial.println(F("Ready. Send: speed,angle"));
}

void loop() {
  static unsigned long lastSend = 0;

  readSerial();

  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    sendPacket();
  }
}