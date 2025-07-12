#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT Nano Ultrasonico ===
#define TRIG_PIN      4    // Trigger del HC-SR04
#define ECHO_PIN      5    // Echo del HC-SR04
#define DE_RE_PIN     8    // DE & RE del MAX485 (puentes juntos)
#define RX_PIN       10    // RO del MAX485 (no usado internamente)
#define TX_PIN       11    // DI del MAX485
#define NODE_ID     109   // ID único para Ultrasonico

// Constantes
const float SOUND_CM_PER_US = 0.0343 / 2.0;  // velocidad sonido: 343 m/s
const int   BUF_SIZE       = 60;            // muestras para 60 s
float       bufDist[BUF_SIZE];
uint8_t     idxDist   = 0;
bool        fullDist  = false;
float       sumDist   = 0;

// Tiempos
unsigned long t1_sec  = 0;
unsigned long t60_sec = 0;

// RS-485 por SoftwareSerial
SoftwareSerial rs485(RX_PIN, TX_PIN);
// LCD I2C 16×2 a 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Lee distancia en cm
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms
  return dur * SOUND_CM_PER_US;
}

void setup() {
  Serial.begin(9600);
  rs485.begin(9600);

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);   // RX por defecto

  // Pines HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Ultrasonico...");
  delay(1500);
  lcd.clear();

  // Inicializar buffer y timers
  for(int i = 0; i < BUF_SIZE; i++) bufDist[i] = 0;
  t1_sec  = millis();
  t60_sec = millis();
}

void loop() {
  unsigned long now = millis();
  static float lastDist = 0;

  // --- Cada 1 s: lectura instantánea y buffer ---
  if(now - t1_sec >= 1000) {
    t1_sec += 1000;
    float d = readUltrasonic();
    lastDist = d;

    // Buffer circular
    if(fullDist) sumDist -= bufDist[idxDist];
    bufDist[idxDist] = d;
    sumDist += d;
    if(++idxDist == BUF_SIZE) {
      idxDist = 0;
      fullDist = true;
    }

    // Promedio parcial
    uint8_t cnt = fullDist ? BUF_SIZE : idxDist;
    float avg = cnt ? sumDist / cnt : 0;

    // Mostrar en LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Inst:"); lcd.print(d,1); lcd.print("cm");
    lcd.setCursor(0, 1);
    lcd.print("Avg: "); lcd.print(avg,1); lcd.print("cm");

    Serial.print("Inst cm: "); Serial.print(d,1);
    Serial.print("  Avg cm: "); Serial.println(avg,1);
  }

  // --- Cada 60 s: enviar promedio ---
  if(now - t60_sec >= 60000) {
    t60_sec += 60000;
    uint8_t cnt = fullDist ? BUF_SIZE : idxDist;
    if(cnt == 0) return;
    float avg = sumDist / cnt;

    // Trama: i<ID>:<valor>f
    String msg = "i";
    msg += NODE_ID;
    msg += ":";
    msg += String(avg, 1);
    msg += "f";

    digitalWrite(DE_RE_PIN, HIGH);
    delayMicroseconds(10);
    rs485.print(msg);
    rs485.flush();
    delayMicroseconds(10);
    digitalWrite(DE_RE_PIN, LOW);

    Serial.print("Enviado "); Serial.println(msg);
  }
}
