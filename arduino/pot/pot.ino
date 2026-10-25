#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

// ---------- Pot ID ----------
#define POT_ID 1   // CHANGE FOR EACH POT: 1, 2, 3, ... (max 99)

// ---------- Pins ----------
#define relay    8
#define soil     A1
#define light    A0
#define DHT_PIN  3
#define LED_PIN  4
// OLED: SDA -> A4, SCL -> A5
// nRF24: CE -> D9, CSN -> D10, SCK -> D13, MOSI -> D11, MISO -> D12, VCC -> 3.3V

#define RELAY_ON  HIGH
#define RELAY_OFF LOW
#define NUM_LEDS  10
#define DHT_TYPE  DHT11
const bool USE_OLED = false;   // set to true when the new OLED is installed

const unsigned long MAX_PULSE_MS = 15000;   // hard safety limit for one pump run

// ---------- Settings (changed from the website, saved in EEPROM) ----------
#define SETTINGS_MAGIC 0x5347
struct Settings {
  uint16_t magic;
  uint8_t  autoWater;
  uint8_t  dryPct;        // start watering below this soil %
  uint8_t  wetPct;        // stop watering at this soil %
  uint8_t  maxPulses;
  uint32_t pulseMs;
  uint32_t soakMs;
  uint32_t checkMs;
  uint8_t  lightMode;     // 0 auto, 1 always on, 2 always off
  uint8_t  lightOnPct;    // auto: turn on below this light %
  uint8_t  lightOffPct;   // auto: turn off above this light %
  uint8_t  brightness;
  uint32_t color;         // 0xRRGGBB
  int16_t  soilRawDry;
  int16_t  soilRawWet;
  int16_t  lightRawDark;
  int16_t  lightRawBright;
};
Settings cfg;

// ---------- Radio packets (must be above the first function) ----------
struct __attribute__((packed)) Packet {
  uint8_t  magic, version, type, potId;
  uint16_t seq;
  float    temp, hum;
  int16_t  soil, light;         // %
  int16_t  rawSoil, rawLight;   // raw readings, for calibration
  uint8_t  water, grow, pulses, autoWater;
};

struct __attribute__((packed)) Cmd {
  uint8_t  magic, version, type, potId;
  uint16_t cmdSeq;
  uint8_t  key;
  int32_t  value;
};

void loadDefaults() {
  cfg.magic = SETTINGS_MAGIC;
  cfg.autoWater = 1;
  cfg.dryPct = 33;  cfg.wetPct = 65;  cfg.maxPulses = 5;
  cfg.pulseMs = 3000;  cfg.soakMs = 60000;  cfg.checkMs = 10000;
  cfg.lightMode = 0;  cfg.lightOnPct = 70;  cfg.lightOffPct = 80;
  cfg.brightness = 200;  cfg.color = 0xFF0078;
  cfg.soilRawDry = 577;  cfg.soilRawWet = 280;
  cfg.lightRawDark = 1023;  cfg.lightRawBright = 0;
}

// Fix any out-of-range values left in EEPROM by older versions
void sanitizeSettings() {
  if (cfg.pulseMs < 200 || cfg.pulseMs > MAX_PULSE_MS)    cfg.pulseMs = 3000;
  if (cfg.soakMs < 1000 || cfg.soakMs > 3600000UL)        cfg.soakMs = 60000;
  if (cfg.checkMs < 1000 || cfg.checkMs > 3600000UL)      cfg.checkMs = 10000;
  if (cfg.maxPulses < 1 || cfg.maxPulses > 50)            cfg.maxPulses = 5;
  if (cfg.dryPct > 100) cfg.dryPct = 33;
  if (cfg.wetPct > 100) cfg.wetPct = 65;
  if (cfg.lightMode > 2) cfg.lightMode = 0;
}

void loadSettings() {
  EEPROM.get(0, cfg);
  if (cfg.magic != SETTINGS_MAGIC) loadDefaults();
  sanitizeSettings();
  EEPROM.put(0, cfg);
}

void saveSettings() { EEPROM.put(0, cfg); }

// ---------- Watering ----------
enum WaterState { IDLE, WATERING, SOAKING };
WaterState waterState = IDLE;
unsigned long waterTimer = 0;
int pulses = 0;

// ---------- Grow light ----------
const unsigned long LIGHT_CHECK_MS = 5000;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
bool growOn = false;
unsigned long lightTimer = 0;

// ---------- Temp / humidity ----------
const unsigned long DHT_MS = 5000;
DHT dht(DHT_PIN, DHT_TYPE);
unsigned long dhtTimer = 0;

// ---------- OLED ----------
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
const unsigned long OLED_MS = 1000;
unsigned long oledTimer = 0;
int lastSoil = 0, lastLight = 0;
float lastT = NAN, lastH = NAN;

// ---------- Radio / protocol ----------
RF24 radio(9, 10);   // CE, CSN
const byte hubAddr[6] = "GARDN";
byte myAddr[6] = {'P', 'O', 'T', '0' + POT_ID / 10, '0' + POT_ID % 10, 0};

#define PROTO_MAGIC   0xA5
#define PROTO_VERSION 2
#define TYPE_STATUS   1
#define TYPE_CMD      2

// Setting keys
#define K_AUTO_WATER        1
#define K_DRY_PCT           2
#define K_WET_PCT           3
#define K_PULSE_MS          4
#define K_SOAK_MS           5
#define K_CHECK_MS          6
#define K_MAX_PULSES        7
#define K_LIGHT_MODE        8
#define K_LIGHT_ON_PCT      9
#define K_LIGHT_OFF_PCT    10
#define K_BRIGHTNESS       11
#define K_COLOR            12
#define K_SOIL_RAW_DRY     13
#define K_SOIL_RAW_WET     14
#define K_LIGHT_RAW_DARK   15
#define K_LIGHT_RAW_BRIGHT 16
// Actions
#define A_WATER_NOW       100
#define A_STOP_WATER      101

const unsigned long RADIO_MS  = 2000;
const unsigned long LISTEN_MS = 300;   // listen for commands after each report
unsigned long radioTimer = 0;
unsigned long listenStart = 0;
bool listening = false;
uint16_t seq = 0;
uint16_t lastCmdSeq = 0xFFFF;
uint8_t failCount = 0;

// ---------- Helpers ----------
int toPercent(int raw, int rawAt0, int rawAt100) {
  if (rawAt0 == rawAt100) return 0;
  long p = (long)(raw - rawAt0) * 100 / (rawAt100 - rawAt0);
  return constrain(p, 0, 100);
}

int readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(pin); delay(2); }
  return sum / 10;
}

void setGrow(bool on) {
  if (on) strip.fill(strip.Color((cfg.color >> 16) & 0xFF, (cfg.color >> 8) & 0xFF, cfg.color & 0xFF));
  else    strip.clear();
  strip.show();
}

void showWatering() {
  if (!USE_OLED) return;
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_ncenB14_tr);
    oled.setCursor(10, 38);
    oled.print("Watering...");
  } while (oled.nextPage());
}

void startWatering(unsigned long now) {
  pulses = 0;
  showWatering();
  digitalWrite(relay, RELAY_ON);
  waterState = WATERING;
  waterTimer = now;
  Serial.println("-> Watering, pulse 1");
}

void stopWatering(unsigned long now) {
  digitalWrite(relay, RELAY_OFF);
  waterState = IDLE;
  waterTimer = now;
  Serial.println("-> Watering stopped");
}

void setupRadio() {
  if (!radio.begin()) Serial.println("nRF not found, check wiring");
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(3 + (POT_ID % 10), 15);
  radio.openWritingPipe(hubAddr);
  radio.openReadingPipe(1, myAddr);
  radio.stopListening();
  listening = false;
}

void applyCmd(const Cmd& c, unsigned long now) {
  if (c.magic != PROTO_MAGIC || c.version != PROTO_VERSION) return;
  if (c.type != TYPE_CMD || c.potId != POT_ID) return;
  if (c.cmdSeq == lastCmdSeq) return;   // duplicate delivery
  lastCmdSeq = c.cmdSeq;
  long v = c.value;
  Serial.print("Command key "); Serial.print(c.key);
  Serial.print(" = "); Serial.println(v);

  switch (c.key) {
    case K_AUTO_WATER:       cfg.autoWater = v ? 1 : 0; break;
    case K_DRY_PCT:          cfg.dryPct = constrain(v, 0, 100); break;
    case K_WET_PCT:          cfg.wetPct = constrain(v, 0, 100); break;
    case K_PULSE_MS:         cfg.pulseMs = constrain(v, 200, (long)MAX_PULSE_MS); break;
    case K_SOAK_MS:          cfg.soakMs = constrain(v, 1000, 3600000L); break;
    case K_CHECK_MS:         cfg.checkMs = constrain(v, 1000, 3600000L); break;
    case K_MAX_PULSES:       cfg.maxPulses = constrain(v, 1, 50); break;
    case K_LIGHT_MODE:       cfg.lightMode = (v >= 0 && v <= 2) ? v : 0; break;
    case K_LIGHT_ON_PCT:     cfg.lightOnPct = constrain(v, 0, 100); break;
    case K_LIGHT_OFF_PCT:    cfg.lightOffPct = constrain(v, 0, 100); break;
    case K_BRIGHTNESS:       cfg.brightness = constrain(v, 0, 255);
                             strip.setBrightness(cfg.brightness); break;
    case K_COLOR:            cfg.color = v & 0xFFFFFF; break;
    case K_SOIL_RAW_DRY:     cfg.soilRawDry = constrain(v, 0, 1023); break;
    case K_SOIL_RAW_WET:     cfg.soilRawWet = constrain(v, 0, 1023); break;
    case K_LIGHT_RAW_DARK:   cfg.lightRawDark = constrain(v, 0, 1023); break;
    case K_LIGHT_RAW_BRIGHT: cfg.lightRawBright = constrain(v, 0, 1023); break;
    case A_WATER_NOW:        startWatering(now); return;
    case A_STOP_WATER:       stopWatering(now); return;
    default: return;
  }
  saveSettings();
  if (growOn) setGrow(true);           // show new color / brightness right away
  lightTimer = now - LIGHT_CHECK_MS;   // re-check the light on the next loop
}

// ---------- Tasks ----------
void waterTask(unsigned long now) {
  switch (waterState) {
    case IDLE:
      if (now - waterTimer >= cfg.checkMs) {
        waterTimer = now;
        int val = readAvg(soil);
        lastSoil = val;
        int pct = toPercent(val, cfg.soilRawDry, cfg.soilRawWet);
        Serial.print("Soil: "); Serial.print(val);
        Serial.print(" ("); Serial.print(pct); Serial.println(" %)");
        if (cfg.autoWater && pct < cfg.dryPct) startWatering(now);
      }
      break;

    case WATERING:
      if (now - waterTimer >= cfg.pulseMs) {
        digitalWrite(relay, RELAY_OFF);
        if (USE_OLED) { oled.begin(); oledTimer = 0; }
        pulses++;
        waterTimer = now;
        waterState = SOAKING;
        Serial.println("-> Pump off, soaking");
      }
      break;

    case SOAKING:
      if (now - waterTimer >= cfg.soakMs) {
        int val = readAvg(soil);
        lastSoil = val;
        int pct = toPercent(val, cfg.soilRawDry, cfg.soilRawWet);
        Serial.print("Soil after soak: "); Serial.println(pct);
        waterTimer = now;
        if (pct >= cfg.wetPct) {
          waterState = IDLE;
          Serial.println("-> Wet enough, done");
        } else if (pulses >= cfg.maxPulses) {
          waterState = IDLE;
          Serial.println("-> Max pulses hit, check sensor/pump");
        } else {
          showWatering();
          digitalWrite(relay, RELAY_ON);
          waterState = WATERING;
          Serial.print("-> Still dry, pulse "); Serial.println(pulses + 1);
        }
      }
      break;
  }
}

void lightTask(unsigned long now) {
  if (now - lightTimer < LIGHT_CHECK_MS) return;
  lightTimer = now;

  // Blank the strip so the LDR measures real ambient light, not the grow light
  if (growOn) { strip.clear(); strip.show(); delay(30); }
  int val = readAvg(light);
  lastLight = val;
  int pct = toPercent(val, cfg.lightRawDark, cfg.lightRawBright);

  bool want = growOn;
  if (cfg.lightMode == 1)                     want = true;
  else if (cfg.lightMode == 2)                want = false;
  else if (!growOn && pct < cfg.lightOnPct)   want = true;
  else if (growOn && pct > cfg.lightOffPct)   want = false;

  if (want != growOn) {
    growOn = want;
    Serial.println(growOn ? "-> Grow light ON" : "-> Grow light OFF");
  }
  setGrow(growOn);
}

void dhtTask(unsigned long now) {
  if (now - dhtTimer < DHT_MS) return;
  dhtTimer = now;
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println("DHT read failed");
  } else {
    lastT = t;
    lastH = h;
  }
}

void displayTask(unsigned long now) {
  if (!USE_OLED) return;
  if (waterState == WATERING) return;   // don't touch the screen while pumping
  if (now - oledTimer < OLED_MS) return;
  oledTimer = now;

  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    oled.begin();
  }

  const char* ws = waterState == IDLE ? "Idle" : "Soaking";
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_6x10_tf);
    oled.setCursor(0, 10); oled.print("Temp: ");    oled.print(lastT, 1); oled.print(" C");
    oled.setCursor(0, 22); oled.print("Air hum: "); oled.print(lastH, 0); oled.print(" %");
    oled.setCursor(0, 34); oled.print("Soil: ");
    oled.print(toPercent(lastSoil, cfg.soilRawDry, cfg.soilRawWet)); oled.print(" %");
    oled.setCursor(0, 46); oled.print("Light: ");
    oled.print(toPercent(lastLight, cfg.lightRawDark, cfg.lightRawBright)); oled.print(" %");
    oled.print(growOn ? " (grow ON)" : "");
    oled.setCursor(0, 58); oled.print("Water: ");   oled.print(ws);
  } while (oled.nextPage());
}

void radioTask(unsigned long now) {
  // After each report, listen briefly for a command from the hub
  if (listening) {
    if (radio.available()) {
      Cmd c;
      radio.read(&c, sizeof(c));
      radio.stopListening();
      listening = false;
      applyCmd(c, now);
      radioTimer = now - RADIO_MS + 150;   // report again soon to pick up the next command
    } else if (millis() - listenStart >= LISTEN_MS) {
      radio.stopListening();
      listening = false;
    }
    return;
  }

  if (now - radioTimer < RADIO_MS) return;
  radioTimer = now;

  Packet pkt;
  pkt.magic     = PROTO_MAGIC;
  pkt.version   = PROTO_VERSION;
  pkt.type      = TYPE_STATUS;
  pkt.potId     = POT_ID;
  pkt.seq       = seq++;
  pkt.temp      = lastT;
  pkt.hum       = lastH;
  pkt.soil      = toPercent(lastSoil, cfg.soilRawDry, cfg.soilRawWet);
  pkt.light     = toPercent(lastLight, cfg.lightRawDark, cfg.lightRawBright);
  pkt.rawSoil   = lastSoil;
  pkt.rawLight  = lastLight;
  pkt.water     = waterState;
  pkt.grow      = growOn;
  pkt.pulses    = pulses;
  pkt.autoWater = cfg.autoWater;

  if (radio.write(&pkt, sizeof(pkt))) {
    failCount = 0;
  } else {
    Serial.println("Radio send failed");
    if (++failCount >= 3) {      // 3 misses in a row: the nRF probably got scrambled
      Serial.println("Resetting radio");
      setupRadio();
      failCount = 0;
      return;
    }
  }

  radio.startListening();
  listening = true;
  listenStart = millis();   // start the window from the real time the report went out
}

// ---------- Main ----------
void setup() {
  Serial.begin(9600);
  loadSettings();

  pinMode(relay, OUTPUT);
  digitalWrite(relay, RELAY_OFF);
  dht.begin();
  strip.begin();
  strip.setBrightness(cfg.brightness);
  setGrow(false);

  Wire.begin();
  Wire.setWireTimeout(3000, true);
  if (USE_OLED) {
    oled.setBusClock(100000);
    oled.begin();
  }

  setupRadio();
  Serial.print("Pot "); Serial.print(POT_ID); Serial.println(" ready");
}

void loop() {
  unsigned long now = millis();
  waterTask(now);
  lightTask(now);
  dhtTask(now);
  displayTask(now);
  radioTask(now);
}