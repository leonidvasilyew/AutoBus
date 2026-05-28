#include <RF24.h>
#include <Servo.h>

RF24 radio(1, 0);
Servo myservo;
static constexpr uint8_t Address[5] = {0xAB, 0xCD, 0xAB, 0xCD, 0x71};

#define SERVO_PIN 10
static constexpr uint8_t PIN_AIN1 = 4; // WO2
static constexpr uint8_t PIN_AIN2 = 5; // WO1
static constexpr uint8_t PIN_SLEEP = 6;

int16_t speed = 0;
uint8_t angle  = 90;

static unsigned long lastPacket = 0;
const unsigned long FAILSAFE_MS = 500;

static inline void motorInit() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_SLEEP, HIGH);

  takeOverTCA0();
  TCA0.SPLIT.CTRLD &= ~TCA_SPLIT_SPLITM_bm;

  // WO2 -> alt (pin 4), WO1 -> alt (pin 5)
  PORTMUX.CTRLC |= PORTMUX_TCA02_bm | PORTMUX_TCA01_bm;

  TCA0.SINGLE.CTRLB =
    TCA_SINGLE_WGMODE_SINGLESLOPE_gc |
    TCA_SINGLE_CMP2EN_bm |
    TCA_SINGLE_CMP1EN_bm;

  TCA0.SINGLE.PER  = 255;
  TCA0.SINGLE.CMP2 = 0;
  TCA0.SINGLE.CMP1 = 0;

  TCA0.SINGLE.CTRLA =
    TCA_SINGLE_CLKSEL_DIV1_gc |
    TCA_SINGLE_ENABLE_bm;
}

static inline void motorSet(int16_t pwm) {
  // -------- BRAKE --------
  if (pwm > 255 || pwm < -255) {
    TCA0.SINGLE.CMP2 = 255;
    TCA0.SINGLE.CMP1 = 255;
    return;
  }

  // -------- COAST --------
  if (pwm == 0) {
    TCA0.SINGLE.CMP2 = 0;
    TCA0.SINGLE.CMP1 = 0;
    return;
  }

  // -------- DRIVE --------
  if (pwm > 0) {
    // forward
    TCA0.SINGLE.CMP2 = (uint8_t)pwm; // AIN1 = PWM
    TCA0.SINGLE.CMP1 = 0;          // AIN2 = 0
  } else {
    // reverse
    pwm = -pwm;
    TCA0.SINGLE.CMP2 = 0;          // AIN1 = 0
    TCA0.SINGLE.CMP1 = (uint8_t)pwm; // AIN2 = PWM
  }
}

void setup() {
  motorInit();
  myservo.attach(SERVO_PIN);

  radio.begin();
  radio.setChannel(101);
  radio.setPayloadSize(2);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.openReadingPipe(1, Address);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    uint8_t buffer[2];
    radio.read(buffer, sizeof(buffer));
    speed = constrain((int16_t)(int8_t)buffer[0] * 2, -255, 255);
    angle = buffer[1];
    motorSet(speed);
    myservo.write(angle);
    lastPacket = millis();
  }

  if (millis() - lastPacket > FAILSAFE_MS) {
    motorSet(0);
  }
}
