# Smart Garden

Automatic plant care with remote control from a website and a flex-sensor glove.

- **Pots** (Arduino Nano): soil moisture, light, temperature/humidity, water pump, grow light. Talks to the hub over nRF24.
- **Hub** (Arduino Nano): collects every pot over nRF24, shows status on an LCD, connects to the PC over Bluetooth (HC-06).
- **Bridge** (`bridge.py`): PC program linking the hub to Firebase, plus the glove over Bluetooth.
- **Website** (`public/index.html`): live dashboard, history charts, and controls (Firebase Hosting + Realtime Database).
- **Glove** (Arduino Nano + 3 flex sensors + HC-06): one finger per pot, gestures to water, stop, and toggle the light.

## Setup
1. Upload `arduino/pot` to each pot (change `POT_ID`), `arduino/hub` to the hub, `arduino/glove` to the glove.
2. `pip install pyserial firebase-admin`
3. Put your Firebase admin key next to `bridge.py` (it is git-ignored) and set `KEY_FILE`, `PORT`, `GLOVE_PORT`.
4. `python bridge.py`
5. `firebase deploy --only hosting`

Wiring diagrams: see docs/WIRING.md
