#include <SoftwareSerial.h>

// ---------- Pins ----------
#define NUM_FINGERS 3
const uint8_t FLEX_PINS[NUM_FINGERS] = {A0, A1, A2};   // thumb, index, middle
const char* FINGER_NAMES[NUM_FINGERS] = {"Thumb", "Index", "Middle"};
SoftwareSerial bt(2, 3);   // RX (to HC-05 TXD), TX (to HC-05 RXD through divider)
#define LED_PIN 13

// ---------- Gestures (bit 0 = thumb, bit 1 = index, bit 2 = middle) ----------
#define M_ALL   0x07   // all three: water now
#define M_STOP  0x06   // index + middle: stop watering
#define M_LIGHT 0x03   // thumb + index: toggle grow light

const unsigned long SELECT_HOLD_MS = 500;   // single finger
const unsigned long ACTION_HOLD_MS = 800;   // combos

int openVal[NUM_FINGERS], bentVal[NUM_FINGERS];
bool bent[NUM_FINGERS];
uint8_t selectedPot = 1;
uint8_t lastMask = 0;
unsigned long maskSince = 0;
bool armed = true;

int readFlex(uint8_t i) {
  long s = 0;
  for (int k = 0; k < 8; k++) s += analogRead(FLEX_PINS[i]);
  return s / 8;
}

void capture(int* out) {
  long sum[NUM_FINGERS] = {0};
  for (int n = 0; n < 50; n++) {
    for (uint8_t i = 0; i < NUM_FINGERS; i++) sum[i] += readFlex(i);
    delay(20);
  }
  for (uint8_t i = 0; i < NUM_FINGERS; i++) out[i] = sum[i] / 50;
}

void blink(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  delay(ms);
  }
}

// Runs at power-up: open hand, then bend all three fingers
void calibrate() {
  Serial.println("Calibrating: hold your hand OPEN");
  blink(3, 200);
  digitalWrite(LED_PIN, HIGH); delay(800);
  capture(openVal);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Now BEND thumb, index and middle");
  blink(6, 100);
  digitalWrite(LED_PIN, HIGH); delay(800);
  capture(bentVal);
  digitalWrite(LED_PIN, LOW);

  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    Serial.print(FINGER_NAMES[i]); Serial.print(": open ");
    Serial.print(openVal[i]); Serial.print(", bent "); Serial.print(bentVal[i]);
    if (abs(bentVal[i] - openVal[i]) < 30) Serial.print("  <-- not detected, check wiring");
    Serial.println();
  }
}

uint8_t readMask() {
  uint8_t m = 0;
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    int range = bentVal[i] - openVal[i];
    if (abs(range) < 30) { bent[i] = false; continue; }   // sensor not working
    float pct = (float)(readFlex(i) - openVal[i]) / range;
    if (!bent[i] && pct > 0.6) bent[i] = true;            // hysteresis so it doesn't flicker
    else if (bent[i] && pct < 0.4) bent[i] = false;
    if (bent[i]) m |= (1 << i);
  }
  return m;
}

void sendCmd(char type, uint8_t pot) {
  bt.print(type); bt.print(','); bt.println(pot);
  Serial.print("Sent "); Serial.print(type); Serial.print(" for pot "); Serial.println(pot);
}

void onGesture(uint8_t m) {
  if (m == M_ALL)        sendCmd('W', selectedPot);   // water now
  else if (m == M_STOP)  sendCmd('X', selectedPot);   // stop watering
  else if (m == M_LIGHT) sendCmd('T', selectedPot);   // toggle grow light
  else {
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
      if (m == (1 << i)) {
        selectedPot = i + 1;
        sendCmd('P', selectedPot);                    // select pot
      }
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  for (uint8_t i = 0; i < NUM_FINGERS; i++) pinMode(FLEX_PINS[i], INPUT_PULLUP);
  Serial.begin(9600);
  bt.begin(9600);
  calibrate();
  Serial.println("Glove ready");
}

void loop() {
  uint8_t m = readMask();
  unsigned long now = millis();

  if (m != lastMask) { lastMask = m; maskSince = now; }
  if (m == 0) { armed = true; delay(20); return; }   // open hand re-arms
  if (!armed) { delay(20); return; }

  bool single = (m & (m - 1)) == 0;
  bool known = single || m == M_ALL || m == M_STOP || m == M_LIGHT;
  unsigned long need = single ? SELECT_HOLD_MS : ACTION_HOLD_MS;

  if (known && now - maskSince >= need) {
    onGesture(m);
    armed = false;
    blink(1, 60);
  }
  delay(20);
}