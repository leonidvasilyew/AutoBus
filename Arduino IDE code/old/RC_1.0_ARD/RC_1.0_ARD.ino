#include <RF24.h>
#include <PID_v1.h>

RF24 radio(7, 8);
static constexpr uint8_t connectAddress[5] = {0xB0, 0xAD, 0x55, 0xBA, 0x5E};
static constexpr uint8_t baseAddress[5] = {0xB2, 0xAD, 0x55, 0xBA, 0x5E};
static constexpr uint8_t myAddress[5] = {0x71, 0xAD, 0x00, 0x04, 0x7A};

static constexpr uint8_t AIN1_PIN = 9;
static constexpr uint8_t AIN2_PIN = 10;
static constexpr uint16_t TOP_20KHZ = 818;
static constexpr uint8_t FB_PIN = 2;

volatile bool sendDataFlag = false;
volatile unsigned long lastTrigTime = 0;
volatile unsigned long currentPeriod = 0;
unsigned long lastChange = 0;
unsigned long lastNull = 0;
unsigned long lastSend = 0;

float pwm_target = 0;
int pwm = 0;
uint8_t smooth;

static inline void motorInit() {
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  // Fast PWM, TOP = ICR1 (mode 14)
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // non-inverting on OC1A/OC1B
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  ICR1 = TOP_20KHZ;

  // start with 0% duty
  OCR1A = 0;
  OCR1B = 0;

  // prescaler = 1
  TCCR1B |= (1 << CS10);
}

static inline void motorSet(int16_t pwm) {
  // -------- BRAKE --------
  if (pwm > 255 || pwm < -255) {
    OCR1A = TOP_20KHZ; // AIN1 = 1
    OCR1B = TOP_20KHZ; // AIN2 = 1
    return;
  }

  // -------- COAST --------
  if (pwm == 0) {
    OCR1A = 0; // AIN1 = 0
    OCR1B = 0; // AIN2 = 0
    return;
  }

  uint16_t mag = (pwm < 0) ? (uint16_t)(-pwm) : (uint16_t)pwm;
  // map 0..255 -> 0..TOP
  uint16_t duty = (uint32_t)mag * TOP_20KHZ / 255;

  // -------- DRIVE --------
  if (pwm > 0) {
    // forward: AIN1=PWM, AIN2=0
    OCR1A = duty;
    OCR1B = 0;
  } else {
    // reverse: AIN1=0, AIN2=PWM
    OCR1A = 0;
    OCR1B = duty;
  }
}

void writeRadio() {
  
  uint8_t tx_buffer[13];

  tx_buffer[0] = (int8_t)pwm_target;
  float nset_f = (float)pwm;
  float pwm_f  = (float)smooth;
  float nin_f  = (float)0;

  memcpy(tx_buffer + 1,  &nset_f, 4);
  memcpy(tx_buffer + 5,  &pwm_f,  4);
  memcpy(tx_buffer + 9,  &nin_f,  4);

  radio.stopListening();
  radio.write(tx_buffer, 13);
  radio.startListening();

  lastSend = millis();
}

void setup() {
  motorInit();

  radio.begin();
  radio.setChannel(67);
  radio.enableDynamicPayloads();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.setRetries(15, 15);
  radio.openWritingPipe(connectAddress);
  radio.openReadingPipe(1, myAddress);
  radio.startListening();

  unsigned long lastSend = 0;
  bool connected = 0;
  uint8_t buffer[2] = {255, 255};
  while (!connected) {
    if (millis() - lastSend >= 500) {
      lastSend = millis();
      radio.stopListening();
      radio.write(myAddress, 5);
      radio.startListening();
    }
    if (radio.available()) {
      radio.read(buffer, sizeof(buffer));
      if (buffer[0] != 255 || buffer[1] != 255) connected = 1;
    }
  }
  pwm_target = ((int8_t)buffer[0])*2.0f;
  smooth = buffer[1];

  radio.stopListening();
  radio.openWritingPipe(baseAddress);
  writeRadio();
  radio.startListening();
}

void loop() {
  // RF
  if (radio.available()) {
    uint8_t buffer[2];
    radio.read(buffer, sizeof(buffer));
    pwm_target = ((int8_t)buffer[0])*2.0f;
    smooth = buffer[1];

    writeRadio();
  }

  // n_set
  unsigned long now = millis();
  if (pwm != pwm_target && (now - lastChange) >= (uint32_t)smooth * 1.5) {

    if (pwm < pwm_target) pwm += 1;
    else pwm -= 1;

    if (smooth == 0) pwm = pwm_target;
    
    lastChange = now;
    
    motorSet(pwm);
  }

  if (millis() - lastSend > 500) writeRadio();
}
