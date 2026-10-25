#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// ---------- Modules ----------
RF24 radio(9, 10);                    // CE, CSN
const byte hubAddr[6] = "GARDN";
LiquidCrystal_I2C lcd(0x27, 16, 2);   // try 0x3F if the screen stays blank
SoftwareSerial bt(2, 3);              // PC link: RX (to HC-06 TXD), TX (to HC-06 RXD)
// Glove link: second HC-06, its TXD -> D0 (hardware Serial). Unplug D0 when uploading!

// ---------- Protocol ----------
#define PROTO_MAGIC   0xA5
#define PROTO_VERSION 2
#define TYPE_STATUS   1
#define TYPE_CMD      2

#define K_LIGHT_MODE  8
#define A_WATER_NOW   100
#define A_STOP_WATER  101

struct __attribute__((packed)) Packet {
  uint8_t  magic, version, type, potId;
  uint16_t seq;
  float    temp, hum;
  int16_t  soil, light;
  int16_t  rawSoil, rawLight;
  uint8_t  water, grow, pulses, autoWater;
};

struct __attribute__((packed)) Cmd {
  uint8_t  magic, version, type, potId;
  uint16_t cmdSeq;
  uint8_t  key;
  int32_t  value;
};

// ---------- Pot table ----------
#define MAX_POTS 8                            // raise if you add more pots
const unsigned long TIMEOUT_MS = 10000;       // pot counts as offline after this
const unsigned long LCD_CYCLE_MS = 3000;
const unsigned long RADIO_RESET_MS = 30000;   // reset our radio if every pot goes quiet this long

struct PotState {
  bool seen;
  bool lost;
  unsigned long lastRx;
  Packet data;
};
PotState pots[MAX_POTS + 1];

// One waiting command per pot (from the PC bridge or the glove)
struct Pending {
  bool     active;
  uint16_t seq;
  uint8_t  key;
  int32_t  value;
};
Pending pending[MAX_POTS + 1];

uint8_t lcdPot = 0;
unsigned long lcdTimer = 0;
unsigned long lastAnyRx = 0;
unsigned long lastRadioReset = 0;
bool anySeen = false;
char btLine[48];
uint8_t btLen = 0;

// ---------- Glove ----------
char gloveLine[16];
uint8_t gloveLen = 0;
uint16_t gloveSeq = 50000;
uint8_t gloveMsgPot = 0;
const char* gloveMsg = "";
unsigned long gloveMsgUntil = 0;

// ---------- Helpers ----------
void potAddr(uint8_t id, byte* a) {
  a[0] = 'P'; a[1] = 'O'; a[2] = 'T';
  a[3] = '0' + id / 10; a[4] = '0' + id % 10; a[5] = 0;
}

void setupRadio() {
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, hubAddr);
  radio.startListening();
}

// Reject packets that were scrambled by electrical noise
bool plausible(const Packet& p) {
  return p.soil >= 0 && p.soil <= 100 &&
         p.light >= 0 && p.light <= 100 &&
         p.rawSoil >= 0 && p.rawSoil <= 1023 &&
         p.rawLight >= 0 && p.rawLight <= 1023 &&
         p.water <= 2 && p.grow <= 1 && p.autoWater <= 1 && p.pulses <= 50;
}

// Format: D,pot,temp,hum,soil,light,water,grow,seq,rawSoil,rawLight,pulses,autoWater
void sendBT(const Packet& p) {
  bt.print("D,");  bt.print(p.potId);
  bt.print(',');   bt.print(p.temp, 1);
  bt.print(',');   bt.print(p.hum, 0);
  bt.print(',');   bt.print(p.soil);
  bt.print(',');   bt.print(p.light);
  bt.print(',');   bt.print(p.water);
  bt.print(',');   bt.print(p.grow);
  bt.print(',');   bt.print(p.seq);
  bt.print(',');   bt.print(p.rawSoil);
  bt.print(',');   bt.print(p.rawLight);
  bt.print(',');   bt.print(p.pulses);
  bt.print(',');   bt.println(p.autoWater);
}

// The pot listens for a moment right after it reports, so send now
void deliver(uint8_t id) {
  Pending& pd = pending[id];
  if (!pd.active) return;
  Cmd c = {PROTO_MAGIC, PROTO_VERSION, TYPE_CMD, id, pd.seq, pd.key, pd.value};
  byte a[6];
  potAddr(id, a);
  radio.stopListening();
  radio.openWritingPipe(a);
  bool ok = radio.write(&c, sizeof(c));
  radio.startListening();
  if (ok) {
    pd.active = false;
    bt.print("A,"); bt.print(id); bt.print(','); bt.println(c.cmdSeq);
  }
}

void handlePacket(const Packet& p) {
  if (p.magic != PROTO_MAGIC || p.version != PROTO_VERSION) return;
  if (p.type != TYPE_STATUS) return;
  if (p.potId < 1 || p.potId > MAX_POTS) return;
  if (!plausible(p)) return;

  PotState& s = pots[p.potId];
  s.data = p;
  s.lastRx = millis();
  s.seen = true;
  s.lost = false;
  lastAnyRx = millis();
  anySeen = true;

  deliver(p.potId);   // first, while the pot is still listening
  sendBT(p);
}

// ---------- PC commands: C,pot,cmdSeq,key,value ----------
void parseLine(char* s) {
  if (s[0] != 'C' || s[1] != ',') return;
  char* p = s + 2;
  long f[4];
  for (int i = 0; i < 4; i++) {
    f[i] = strtol(p, &p, 10);
    if (i < 3) {
      if (*p != ',') return;
      p++;
    }
  }
  if (f[0] < 1 || f[0] > MAX_POTS) return;
  pending[f[0]] = {true, (uint16_t)f[1], (uint8_t)f[2], (int32_t)f[3]};
}

void readBT() {
  while (bt.available()) {
    char ch = bt.read();
    if (ch == '\n' || ch == '\r') {
      if (btLen) { btLine[btLen] = 0; parseLine(btLine); btLen = 0; }
    } else if (btLen < sizeof(btLine) - 1) {
      btLine[btLen++] = ch;
    }
  }
}

// ---------- Glove commands: P,pot  W,pot  X,pot  T,pot ----------
void showLCD();

void gloveCommand(char type, uint8_t id) {
  if (id < 1 || id > MAX_POTS) return;
  switch (type) {
    case 'P':
      gloveMsg = "Selected";
      break;
    case 'W':
      pending[id] = {true, gloveSeq++, A_WATER_NOW, 0};
      gloveMsg = "Water now";
      break;
    case 'X':
      pending[id] = {true, gloveSeq++, A_STOP_WATER, 0};
      gloveMsg = "Stop watering";
      break;
    case 'T': {
      bool on = pots[id].data.grow;
      pending[id] = {true, gloveSeq++, K_LIGHT_MODE, on ? 2 : 1};   // 1 = always on, 2 = always off
      gloveMsg = on ? "Light OFF" : "Light ON";
      break;
    }
    default:
      return;
  }
  gloveMsgPot = id;
  gloveMsgUntil = millis() + 2500;
  showLCD();
}

void readGlove() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (gloveLen >= 3 && gloveLine[1] == ',') {
        gloveLine[gloveLen] = 0;
        gloveCommand(gloveLine[0], atoi(gloveLine + 2));
      }
      gloveLen = 0;
    } else if (gloveLen < sizeof(gloveLine) - 1) {
      gloveLine[gloveLen++] = ch;
    }
  }
}

// ---------- Status ----------
void checkTimeouts() {
  for (uint8_t id = 1; id <= MAX_POTS; id++) {
    PotState& s = pots[id];
    if (s.seen && !s.lost && millis() - s.lastRx > TIMEOUT_MS) {
      s.lost = true;
      bt.print("L,"); bt.println(id);
    }
  }

  // If nothing has been heard from any pot for a while, our own radio may be stuck
  if (anySeen && millis() - lastAnyRx > RADIO_RESET_MS &&
      millis() - lastRadioReset > RADIO_RESET_MS) {
    setupRadio();
    lastRadioReset = millis();
  }
}

uint8_t nextSeenPot(uint8_t from) {
  for (uint8_t i = 1; i <= MAX_POTS; i++) {
    uint8_t id = (from + i - 1) % MAX_POTS + 1;
    if (pots[id].seen) return id;
  }
  return 0;
}

void showLCD() {
  lcd.clear();

  // Glove feedback takes over the screen for a moment
  if (millis() < gloveMsgUntil) {
    lcd.print("Glove: Pot "); lcd.print(gloveMsgPot);
    lcd.setCursor(0, 1);
    lcd.print(pots[gloveMsgPot].seen ? gloveMsg : "No such pot");
    return;
  }

  uint8_t seen = 0, online = 0;
  for (uint8_t id = 1; id <= MAX_POTS; id++) {
    if (pots[id].seen) {
      seen++;
      if (!pots[id].lost) online++;
    }
  }
  if (seen == 0 || lcdPot == 0) {
    lcd.print("Waiting for");
    lcd.setCursor(0, 1);
    lcd.print("pots...");
    return;
  }

  lcd.print("Online: "); lcd.print(online);
  lcd.print(" of ");     lcd.print(seen);
  lcd.setCursor(0, 1);
  lcd.print("Pot ");     lcd.print(lcdPot);
  lcd.print(": ");
  lcd.print(pots[lcdPot].lost ? "Offline" : "Online");
}

// ---------- Main ----------
void setup() {
  Serial.begin(9600);   // glove HC-06 on D0
  bt.begin(9600);       // PC HC-06 on D2/D3

  lcd.init();
  lcd.backlight();

  setupRadio();
  showLCD();
}

void loop() {
  while (radio.available()) {
    Packet p;
    radio.read(&p, sizeof(p));
    handlePacket(p);
  }

  readBT();
  readGlove();
  checkTimeouts();

  if (millis() - lcdTimer >= LCD_CYCLE_MS) {
    lcdTimer = millis();
    lcdPot = nextSeenPot(lcdPot);
    showLCD();
  }
}