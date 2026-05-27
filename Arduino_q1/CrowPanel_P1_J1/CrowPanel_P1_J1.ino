#include <Arduino_GFX_Library.h>
#include "icon_j1.h"
#include "icon_j2.h"
#include "icon_j3.h"

// ---- Knob-Auswahl ----
// Nur diesen Wert aendern: 1 = P1/Achse 1, 2 = P2/Achse 2, 3 = P3/Achse 3
#define KNOB_ID 1

#if (KNOB_ID < 1) || (KNOB_ID > 3)
#error "KNOB_ID muss aktuell 1, 2 oder 3 sein."
#endif

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define DEVICE_ID "P" STR(KNOB_ID)
#define AXIS_LABEL "Achse " STR(KNOB_ID)

// ---- Farbkonstanten ----
#define WHITE 0xFFFF
#define BLACK 0x0000

// ---- Pinout CrowPanel ----
#define TFT_SCLK    10
#define TFT_MOSI    11
#define TFT_MISO    -1
#define TFT_DC      3
#define TFT_CS      9
#define TFT_RES     14
#define TFT_BLK     46

#define LCD_PWR_EN1 1
#define LCD_PWR_EN2 2

#define ENCODER_CLK 45
#define ENCODER_DT  42
#define ENCODER_BTN 41

// ---- Display-Objekte ----
Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(
  TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO, FSPI, true
);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RES, 0, true);

// ---- Encoder-State + Beschleunigung ----
volatile int enc_old_state = -1;
volatile long enc_delta_raw = 0;
volatile bool btn_pressed = false;

volatile unsigned long last_step_time_us = 0;
volatile unsigned long last_dt_us = 0;

// ===== Encoder Interrupts =====
void IRAM_ATTR encoder_irq() {
  int state = digitalRead(ENCODER_CLK);
  if (state != enc_old_state) {
    int dir = (digitalRead(ENCODER_DT) == state) ? +1 : -1;

    unsigned long now = micros();
    if (last_step_time_us != 0) last_dt_us = now - last_step_time_us;
    last_step_time_us = now;

    enc_delta_raw += dir;
    enc_old_state = state;
  }
}

void IRAM_ATTR button_irq() {
  if (!digitalRead(ENCODER_BTN)) btn_pressed = true; // active LOW
}

// ===== Beschleunigungslogik =====
long apply_acceleration(long delta, unsigned long dt_us)
{
  if (delta == 0) return 0;

  int factor = 1;
  if (dt_us > 0) {
    if (dt_us < 5000) factor = 5;
    else if (dt_us < 15000) factor = 3;
    else if (dt_us < 40000) factor = 2;
  }
  return delta * factor;
}

void drawKnobIcon() {
  switch (KNOB_ID) {
    case 1:
      gfx->draw16bitRGBBitmap(0, 0, icon_j1, ICON_W, ICON_H);
      break;
    case 2:
      gfx->draw16bitRGBBitmap(0, 0, icon_j2, ICON_W, ICON_H);
      break;
    case 3:
      gfx->draw16bitRGBBitmap(0, 0, icon_j3, ICON_W, ICON_H);
      break;
  }
}

// ===== Display Init =====
void initPower() {
  pinMode(LCD_PWR_EN1, OUTPUT);
  pinMode(LCD_PWR_EN2, OUTPUT);
  digitalWrite(LCD_PWR_EN1, HIGH);
  digitalWrite(LCD_PWR_EN2, HIGH);

  pinMode(TFT_BLK, OUTPUT);
  analogWrite(TFT_BLK, 200);
}

void initDisplay() {
  delay(20);
  gfx->begin();
  gfx->fillScreen(BLACK);

  // Icon zeichnen
  drawKnobIcon();

  // Achsen-Label oben links
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(80, 15);
  gfx->println(AXIS_LABEL);
}

// ===== Encoder Init =====
void initEncoder() {
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT,  INPUT_PULLUP);

  enc_old_state = digitalRead(ENCODER_CLK);

  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoder_irq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BTN), button_irq, CHANGE);
}

// ===== Overlay-Update (Encoder-Werte anzeigen) =====
void updateDeltaOnScreen(long delta_scaled) {
  gfx->fillRect(40, 200, 160, 30, BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(60, 200);
  gfx->printf("dEnc=%d", delta_scaled);
}

// ===== Hauptprogram =====
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.print(DEVICE_ID); Serial.println(":BOOT");

  initPower();
  initEncoder();
  initDisplay();

  Serial.print(DEVICE_ID); Serial.println(":READY");
}

void loop() {
  long delta_raw = 0;
  bool btn = false;
  unsigned long dt_us = 0;

  noInterrupts();
  if (enc_delta_raw != 0) {
    delta_raw = enc_delta_raw;
    enc_delta_raw = 0;
    dt_us = last_dt_us;
  }
  if (btn_pressed) {
    btn = true;
    btn_pressed = false;
  }
  interrupts();

  if (delta_raw != 0) {
    long delta_scaled = apply_acceleration(delta_raw, dt_us);

    Serial.print(DEVICE_ID);
    Serial.print(":ENC:");
    Serial.println(delta_scaled);

    updateDeltaOnScreen(delta_scaled);
  }

  if (btn) {
    Serial.print(DEVICE_ID);
    Serial.println(":BTN:1");
  }

  delay(5);
}
