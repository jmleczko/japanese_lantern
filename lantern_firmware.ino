/*
  Japanese Lantern Firmware - ATtiny85
  =====================================
  Hardware assumed (per the schematic we built earlier):
    PB0  -> White LED channel, MOSFET gate via R7 (220R), PWM output (OC0A)
    PB1  -> Red LED channel,   MOSFET gate via gate resistor,  PWM output (OC0B)
    PB2  -> TTP223 touch sensor OUT (momentary mode), also the wake-from-sleep source
    PB3  -> unused (free)
    PB4  -> unused (free)
    PB5  -> RESET, left untouched so the chip stays reprogrammable via the
            Arduino-as-ISP method

  Pin numbering note: this sketch uses ATTinyCore's default numbering, where
  Arduino pin 0 = physical PB0, pin 1 = PB1, pin 2 = PB2. Double-check this
  matches your selected board/core before uploading - some other ATtiny cores
  number pins differently.

  Gesture behavior (as decided during design):
    Single tap   -> toggle power on/off
    Double tap   -> switch between white and red channel (while on);
                    while off, a double tap also just turns the lantern on
    Hold         -> ramp brightness in one direction while held; the NEXT
                    hold ramps the opposite direction (direction flips each
                    time, it does not reverse mid-hold)

  Power behavior:
    While the lantern is OFF, the chip drops into power-down sleep and wakes
    only on a pin-change interrupt from the touch sensor (microamp-level
    standby current). While the lantern is ON, the chip stays awake, since
    power-down sleep would stop the timer driving the PWM output.

  Persistence:
    The last channel and brightness are written to EEPROM only at the moment
    the lantern is switched off, so a real power loss (battery removed) still
    restores the last-used setting, without wearing out EEPROM from frequent
    writes.
*/

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

// ---------------- Pins ----------------
const uint8_t PIN_WHITE = 0;   // PB0, OC0A
const uint8_t PIN_RED   = 1;   // PB1, OC0B
const uint8_t PIN_TOUCH = 2;   // PB2, PCINT2

// ---------------- Tunable timing ----------------
const uint16_t DEBOUNCE_MS          = 20;   // ignore touch transitions shorter than this
const uint16_t TAP_MAX_MS           = 400;  // press shorter than this = tap; longer = hold
const uint16_t DOUBLE_TAP_WINDOW_MS = 350;  // max gap between tap release and next press to count as double-tap
const uint16_t HOLD_RAMP_INTERVAL_MS= 15;   // how often brightness updates during a hold
const uint8_t  HOLD_RAMP_STEP       = 3;    // brightness change per ramp interval
const uint8_t  MIN_BRIGHTNESS       = 10;   // floor so a "held-down" light never looks fully off
const uint8_t  DEFAULT_BRIGHTNESS   = 160;

// ---------------- EEPROM layout ----------------
// A simple sentinel byte lets us detect "never written before" (fresh chip)
// vs. a previously-saved state, so first boot doesn't load garbage.
const uint8_t EEPROM_MAGIC = 0x5A;
uint8_t EEMEM ee_magic;
uint8_t EEMEM ee_channel;
uint8_t EEMEM ee_brightness;

// ---------------- Runtime state ----------------
enum Channel : uint8_t { CHANNEL_WHITE = 0, CHANNEL_RED = 1 };

bool     powerOn        = false;
Channel  activeChannel  = CHANNEL_WHITE;
uint8_t  brightness     = DEFAULT_BRIGHTNESS;
int8_t   holdDirection  = 1; // flips sign each hold gesture

enum Gesture : uint8_t { GESTURE_NONE, GESTURE_SINGLE_TAP, GESTURE_DOUBLE_TAP, GESTURE_HOLD };

// ---------------- Pin change interrupt ----------------
// This ISR exists only to wake the chip from power-down sleep. No action is
// needed inside it - the rising edge on PB2 is enough to resume execution
// right after sleep_cpu() in goToSleep().
ISR(PCINT0_vect) {
  // intentionally empty
}

void goToSleep() {
  ADCSRA &= ~(1 << ADEN);        // disable ADC before sleeping, saves a little more current
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();
  sleep_cpu();                   // <-- chip halts here until PCINT fires
  sleep_disable();
  ADCSRA |= (1 << ADEN);         // restore ADC in case anything else needs it later
}

// ---------------- LED output ----------------
void updateLEDs() {
  if (!powerOn) {
    analogWrite(PIN_WHITE, 0);
    analogWrite(PIN_RED, 0);
    return;
  }
  if (activeChannel == CHANNEL_WHITE) {
    analogWrite(PIN_WHITE, brightness);
    analogWrite(PIN_RED, 0);
  } else {
    analogWrite(PIN_RED, brightness);
    analogWrite(PIN_WHITE, 0);
  }
}

// ---------------- EEPROM persistence ----------------
void saveState() {
  eeprom_update_byte(&ee_magic, EEPROM_MAGIC);
  eeprom_update_byte(&ee_channel, (uint8_t)activeChannel);
  eeprom_update_byte(&ee_brightness, brightness);
}

void loadState() {
  if (eeprom_read_byte(&ee_magic) == EEPROM_MAGIC) {
    uint8_t ch = eeprom_read_byte(&ee_channel);
    activeChannel = (ch == CHANNEL_RED) ? CHANNEL_RED : CHANNEL_WHITE;
    uint8_t b = eeprom_read_byte(&ee_brightness);
    brightness = (b >= MIN_BRIGHTNESS) ? b : DEFAULT_BRIGHTNESS;
  } else {
    activeChannel = CHANNEL_WHITE;
    brightness = DEFAULT_BRIGHTNESS;
  }
}

// ---------------- Gesture detection ----------------
// Blocks until a full gesture (single tap / double tap / hold) has been
// captured. Brightness ramping for a hold happens live, inside this
// function, while the pin is still held HIGH.
Gesture waitForGesture() {
  // 1) Wait for a touch to begin.
  if (!powerOn) {
    goToSleep(); // wakes on the touch's rising edge
  }
  while (digitalRead(PIN_TOUCH) == LOW) {
    delay(5); // lantern is on, so we stay awake and just poll
  }

  // 2) Debounce the rising edge.
  delay(DEBOUNCE_MS);
  if (digitalRead(PIN_TOUCH) == LOW) {
    return GESTURE_NONE; // it was just a glitch
  }

  unsigned long pressStart = millis();
  bool rampingStarted = false;

  // 3) While held, watch for the hold threshold and ramp brightness live.
  while (digitalRead(PIN_TOUCH) == HIGH) {
    unsigned long heldFor = millis() - pressStart;
    if (heldFor > TAP_MAX_MS && powerOn) {
      rampingStarted = true;
      int16_t next = (int16_t)brightness + (holdDirection * HOLD_RAMP_STEP);
      if (next > 255) next = 255;
      if (next < MIN_BRIGHTNESS) next = MIN_BRIGHTNESS;
      brightness = (uint8_t)next;
      updateLEDs();
      delay(HOLD_RAMP_INTERVAL_MS);
    }
  }

  if (rampingStarted) {
    holdDirection = -holdDirection; // next hold goes the other way
    return GESTURE_HOLD;
  }

  // 4) It was a tap. Check whether a second tap follows within the window.
  unsigned long releaseTime = millis();
  while (millis() - releaseTime < DOUBLE_TAP_WINDOW_MS) {
    if (digitalRead(PIN_TOUCH) == HIGH) {
      delay(DEBOUNCE_MS);
      if (digitalRead(PIN_TOUCH) == HIGH) {
        // consume the second press fully before returning
        while (digitalRead(PIN_TOUCH) == HIGH) { delay(5); }
        return GESTURE_DOUBLE_TAP;
      }
    }
  }
  return GESTURE_SINGLE_TAP;
}

// ---------------- Setup / loop ----------------
void setup() {
  pinMode(PIN_WHITE, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_TOUCH, INPUT);

  loadState();
  powerOn = false; // lantern always boots off; tap to turn on
  updateLEDs();

  // Pin change interrupt setup (wakes the chip from sleep on PB2 activity).
  GIMSK |= (1 << PCIE);
  PCMSK |= (1 << PCINT2);
  sei();
}

void loop() {
  Gesture g = waitForGesture();

  switch (g) {
    case GESTURE_SINGLE_TAP:
      powerOn = !powerOn;
      if (!powerOn) {
        saveState(); // persist across a real power loss
      }
      updateLEDs();
      break;

    case GESTURE_DOUBLE_TAP:
      if (!powerOn) {
        powerOn = true;
      } else {
        activeChannel = (activeChannel == CHANNEL_WHITE) ? CHANNEL_RED : CHANNEL_WHITE;
      }
      updateLEDs();
      break;

    case GESTURE_HOLD:
      // Brightness was already updated live in waitForGesture().
      // Nothing further to do here.
      break;

    case GESTURE_NONE:
    default:
      break;
  }
}
