#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include <RF24.h>

// ---- Радио ----
RF24 radio(2, 4);
static constexpr uint8_t Address[5] = {0xAB, 0xCD, 0xAB, 0xCD, 0x71};

// ---- Wi-Fi AP ----
const char* AP_SSID = "BusRC";
const char* AP_PASS = "12345678";
IPAddress apIP(192, 168, 4, 1);
IPAddress apMask(255, 255, 255, 0);

ESP8266WebServer server(80);

// ---- Общее состояние управления ----
volatile int8_t  g_speed = 0;   // -128..127
volatile uint8_t g_angle = 90;  // 0..180

unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 400;
const unsigned long SEND_INTERVAL_MS = 50;

// Буфер строки для команд из Serial
static char lineBuf[32];
static uint8_t lineLen = 0;

// ---- HTML страница ----
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no, viewport-fit=cover">
<title>Bus RC</title>
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; user-select: none; -webkit-user-select: none; touch-action: none; }
  html, body { margin: 0; padding: 0; height: 100%; background: #111; color: #eee; font-family: -apple-system, system-ui, sans-serif; overflow: hidden; }
  .wrap { height: 100dvh; display: flex; flex-direction: column; }
  .hud { display: flex; justify-content: space-between; padding: 10px 14px; font-size: 14px; color: #9aa; }
  .hud b { color: #fff; }
  .stage {
    flex: 1; 
    position: relative;
    display: flex; align-items: center; justify-content: space-between;
    padding: 0 6vw 80px;
  }

  /* Вертикальный рычаг (газ) */
  .lever-v {
    position: relative;
    width: min(28vw, 120px);
    height: min(70vh, 520px);
    background: linear-gradient(180deg, #1c1c20 0%, #141417 100%);
    border: 2px solid #2a2a30;
    border-radius: 60px;
  }
  .lever-v .center-line {
    position: absolute; left: 8%; right: 8%; top: 50%;
    height: 1px; background: #25252b;
    transform: translateY(-50%);
  }
  .lever-v .knob {
    position: absolute; left: 50%; top: 50%;
    width: 90%; aspect-ratio: 1/1;
    transform: translate(-50%, -50%);
    background: linear-gradient(180deg, #4a90ff, #2563d9);
    border-radius: 50%;
    box-shadow: 0 8px 20px rgba(0,0,0,0.4), inset 0 -4px 10px rgba(0,0,0,0.3);
  }

  /* Горизонтальный рычаг (поворот) */
  .lever-h {
    position: relative;
    width: min(70vw, 520px);
    height: min(28vw, 120px);
    background: linear-gradient(180deg, #1c1c20 0%, #141417 100%);
    border: 2px solid #2a2a30;
    border-radius: 60px;
  }
  .lever-h .center-line {
    position: absolute; top: 8%; bottom: 8%; left: 50%;
    width: 1px; background: #25252b;
    transform: translateX(-50%);
  }
  .lever-h .knob {
    position: absolute; left: 50%; top: 50%;
    height: 90%; aspect-ratio: 1/1;
    transform: translate(-50%, -50%);
    background: linear-gradient(180deg, #4a90ff, #2563d9);
    border-radius: 50%;
    box-shadow: 0 8px 20px rgba(0,0,0,0.4), inset 0 -4px 10px rgba(0,0,0,0.3);
  }

  .stop {
    position: absolute; bottom: 16px; left: 50%; transform: translateX(-50%);
    padding: 12px 28px; background: #c0392b;
    color: #fff; border: none;
    border-radius: 999px; font-size: 16px; font-weight: 600;
  }
  .stop:active { background: #962d22; }
</style>
</head>
<body>
<div class="wrap">
  <div class="hud">
    <div>скорость: <b id="sv">0</b></div>
    <div>угол: <b id="av">90</b></div>
  </div>
  <div class="stage">
    <div class="lever-v" id="leverV">
      <div class="center-line"></div>
      <div class="knob" id="knobV"></div>
    </div>
    <div class="lever-h" id="leverH">
      <div class="center-line"></div>
      <div class="knob" id="knobH"></div>
    </div>
  </div>
  <button class="stop" id="stop">STOP</button>
</div>
<script>
(function(){
  const leverV = document.getElementById('leverV');
  const knobV  = document.getElementById('knobV');
  const leverH = document.getElementById('leverH');
  const knobH  = document.getElementById('knobH');
 
  const sv = document.getElementById('sv');
  const av = document.getElementById('av');
  const stopBtn = document.getElementById('stop');

  const DEADZONE = 0.08;

  let speed = 0;
  let angle = 90;

  let pidV = null;
  let pidH = null;

  function applyDeadzone(v){
    const a = Math.abs(v);
    if (a < DEADZONE) return 0;
    const sign = v < 0 ? -1 : 1;
    return sign * (a - DEADZONE) / (1 - DEADZONE);
  }

  function setKnobV(ny){
    knobV.style.top = (50 + ny*42) + '%';
  }
  function centerV(){
    setKnobV(0);
    speed = 0;
    sv.textContent = speed;
  }
  function handleV(clientY){
    const r = leverV.getBoundingClientRect();
    const cy = r.top + r.height/2;
    let dy = (clientY - cy) / (r.height/2);
    if (dy >  1) dy =  1;
    if (dy < -1) dy = -1;
    setKnobV(dy);
    const sy = applyDeadzone(-dy);
    if (sy === 0) speed = 0;
    else speed = Math.round(sy * (sy > 0 ? 127 : 128));
    if (speed > 127) speed = 127;
    if (speed < -128) speed = -128;
    sv.textContent = speed;
  }

  function setKnobH(nx){
    knobH.style.left = (50 + nx*42) + '%';
  }
  function centerH(){
    setKnobH(0);
    angle = 90;
    av.textContent = angle;
  }
  function handleH(clientX){
    const r = leverH.getBoundingClientRect();
    const cx = r.left + r.width/2;
    let dx = (clientX - cx) / (r.width/2);
    if (dx >  1) dx =  1;
    if (dx < -1) dx = -1;
    setKnobH(dx);
    const sx = applyDeadzone(dx);
    if (sx === 0) angle = 90;
    else angle = Math.round(90 + sx * 90);
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    av.textContent = angle;
  }

  centerV();
  centerH();

  leverV.addEventListener('pointerdown', e => {
    if (pidV !== null) return;
    pidV = e.pointerId;
    leverV.setPointerCapture(e.pointerId);
    handleV(e.clientY);
  });
  leverV.addEventListener('pointermove', e => {
    if (e.pointerId !== pidV) return;
    handleV(e.clientY);
  });
  function releaseV(e){
    if (e.pointerId !== pidV) return;
    try { leverV.releasePointerCapture(pidV); } catch(_) {}
    pidV = null;
    centerV();
  }
  leverV.addEventListener('pointerup', releaseV);
  leverV.addEventListener('pointercancel', releaseV);
  leverV.addEventListener('pointerleave', releaseV);

  leverH.addEventListener('pointerdown', e => {
    if (pidH !== null) return;
    pidH = e.pointerId;
    leverH.setPointerCapture(e.pointerId);
    handleH(e.clientX);
  });
  leverH.addEventListener('pointermove', e => {
    if (e.pointerId !== pidH) return;
    handleH(e.clientX);
  });
  function releaseH(e){
    if (e.pointerId !== pidH) return;
    try { leverH.releasePointerCapture(pidH); } catch(_) {}
    pidH = null;
    centerH();
  }
  leverH.addEventListener('pointerup', releaseH);
  leverH.addEventListener('pointercancel', releaseH);
  leverH.addEventListener('pointerleave', releaseH);

  stopBtn.addEventListener('click', () => { centerV(); centerH(); });

  let inflight = false;
  setInterval(async () => {
    if (inflight) return;
    inflight = true;
    try {
      await fetch('/c?s=' + speed + '&a=' + angle, { cache: 'no-store' });
    } catch(_) {}
    inflight = false;
  }, 80);
})();
</script>
</body>
</html>
)HTML";

// ---- Разбор строки из Serial ----
static void parseLine(const char* s) {
  // Формат: "speed,angle" (например, "50,110")
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

  // Ограничение диапазонов
  if (sp < -128) sp = -128;
  if (sp >  127) sp =  127;
  if (an < 0)    an = 0;
  if (an > 180)  an = 180;

  g_speed = (int8_t)sp;
  g_angle = (uint8_t)an;
  lastCmdMs = millis(); // Сбрасываем таймаут fail-safe

  Serial.print(F("OK Serial speed="));
  Serial.print(g_speed);
  Serial.print(F(" angle="));
  Serial.println(g_angle);
}

// ---- Чтение Serial портов ----
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
      lineLen = 0;
      Serial.println(F("ERR: line too long"));
    }
  }
}

// ---- Хэндлеры Веб-сервера ----
void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleCmd() {
  if (server.hasArg("s")) {
    long s = server.arg("s").toInt();
    if (s < -128) s = -128;
    if (s >  127) s =  127;
    g_speed = (int8_t)s;
  }
  if (server.hasArg("a")) {
    long a = server.arg("a").toInt();
    if (a < 0)   a = 0;
    if (a > 180) a = 180;
    g_angle = (uint8_t)a;
  }
  lastCmdMs = millis(); // Сбрасываем таймаут fail-safe
  server.send(200, "text/plain", "ok");
}

void handleNotFound() {
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);

  // Настройка Wi-Fi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, apMask);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // Настройка Веб-сервера
  server.on("/", handleRoot);
  server.on("/c", handleCmd);
  server.onNotFound(handleNotFound);
  server.begin();

  // Настройка радио NRF24
  SPI.begin();
  radio.begin();
  radio.setChannel(101);
  radio.setPayloadSize(2);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.setRetries(3, 3);
  radio.openWritingPipe(Address);
  radio.stopListening();

  Serial.println(F("Ready. Listening for Web and Serial (speed,angle)"));
}

// ---- Loop ----
void loop() {
  // 1. Обработка веб-запросов от телефона
  server.handleClient();

  // 2. Обработка команд из Serial-порта
  readSerial();

  // 3. Fail-safe (если нет команд ни от веб, ни от serial — стоп)
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    g_speed = 0;
    g_angle = 90;
  }

  // 4. Отправка пакета по NRF24 каждые 50 мс
  static unsigned long lastSend = 0;
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    int8_t speedVal = (int8_t)g_speed;
    
    // Масштабирование скорости в диапазоны, заложенные в BasePhoneRC
    if (speedVal > 0) {
      speedVal = map(speedVal, 1, 127, 90, 127);
    } else if (speedVal < 0) {
      speedVal = map(speedVal, -1, -128, -90, -128);
    }

    uint8_t buf[2];
    buf[0] = (uint8_t)speedVal; 
    buf[1] = g_angle;
    
    radio.write(buf, sizeof(buf));
  }
}