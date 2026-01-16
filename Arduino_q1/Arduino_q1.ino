#define DEVICE_ID "P1"        // P1 für Panel 1, P2 für Panel 2

#define ENCODER_A_PIN 45      // A / CLK
#define ENCODER_B_PIN 42      // B / DT
#define ENCODER_SW_PIN 41     // Drucktaster

volatile int enc_state = 0;
volatile int enc_old_state = -1;
volatile long enc_delta_raw = 0;

volatile bool btn_event = false;
volatile unsigned long last_step_time_us = 0;
volatile unsigned long last_dt_us = 0;

void IRAM_ATTR encoder_isr() {
  int a = digitalRead(ENCODER_A_PIN);
  if (a != enc_old_state) {
    int b = digitalRead(ENCODER_B_PIN);
    int dir = (b == a) ? -1 : +1;

    unsigned long now = micros();
    if (last_step_time_us != 0) {
      last_dt_us = now - last_step_time_us;
    }
    last_step_time_us = now;

    enc_delta_raw += dir;
    enc_old_state = a;
  }
}

void IRAM_ATTR button_isr() {
  btn_event = true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN), button_isr, FALLING);

  Serial.print(DEVICE_ID);
  Serial.println(":READY");
}

long apply_acceleration(long delta, unsigned long dt_us) {
  if (delta == 0) return 0;

  // einfache Schwellenwerte – kannst du nach Geschmack tunen
  int factor = 1;
  if (dt_us > 0) {
    if (dt_us < 5000) {          // < 5 ms zwischen Schritten → sehr schnell
      factor = 5;
    } else if (dt_us < 15000) {  // 5–15 ms → schnell
      factor = 3;
    } else if (dt_us < 40000) {  // 15–40 ms → mittel
      factor = 2;
    } else {
      factor = 1;                // langsam
    }
  }

  long sgn = (delta > 0) ? 1 : -1;
  long mag = labs(delta);
  return sgn * mag * factor;
}

void loop() {
  noInterrupts();
  long delta_raw = enc_delta_raw;
  enc_delta_raw = 0;
  bool btn = btn_event;
  btn_event = false;
  unsigned long dt_us = last_dt_us;
  interrupts();

  if (delta_raw != 0) {
    long delta_scaled = apply_acceleration(delta_raw, dt_us);
    Serial.print(DEVICE_ID);
    Serial.print(":ENC:");
    Serial.println(delta_scaled);
    // Wenn du Debug willst:
    // Serial.print(DEVICE_ID); Serial.print(":RAW:"); Serial.print(delta_raw);
    // Serial.print(" DT(us)="); Serial.println(dt_us);
  }

  if (btn) {
    Serial.print(DEVICE_ID);
    Serial.println(":BTN:1");
  }

  delay(5);
}
