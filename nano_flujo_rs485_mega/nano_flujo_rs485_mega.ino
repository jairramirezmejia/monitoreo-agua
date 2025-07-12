#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT Nano Flujo con LCD 16×2 ===
#define FLOW_PIN      2    // Sensor YF-S201 → interrupción
#define DE_RE_PIN     8    // DE & RE del MAX485 (puentes juntos)
#define RX_PIN       10    // RO del MAX485 (no se usa para lectura interna)
#define TX_PIN       11   // DI del MAX485
#define NODE_ID     108   // ID único para flujo en la trama

// Pulsos por litro (YF-S201 ≈ 450 pulses/L)
const float PULSES_PER_L = 450.0;

// Buffer para promedio de 60 muestras (1 Hz durante 60 s)
float bufFlow[60];
uint8_t idxFlow   = 0;
bool    fullFlow  = false;
float   sumFlow   = 0;

// Tiempos para muestreo
unsigned long t1_sec  = 0;
unsigned long t60_sec = 0;

// Contador de pulsos
volatile uint16_t pulseCount = 0;

// RS-485 por SoftwareSerial
SoftwareSerial rs485(RX_PIN, TX_PIN);

// LCD I2C 16×2 en dirección 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ISR: contar cada pulso del sensor
void isrFlow() {
  pulseCount++;
}

void setup() {
  // Debug por USB
  Serial.begin(9600);

  // Inicializa RS-485
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);    // RX por defecto
  rs485.begin(9600);

  // Configura sensor de flujo
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), isrFlow, RISING);

  // Inicializa buffer
  for (int i = 0; i < 60; i++) bufFlow[i] = 0;
  t1_sec  = millis();
  t60_sec = millis();

  // Inicializa LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Flujo: Inicializ");
  delay(1500);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();
  static float lastLpm = 0;

  // === Cada 1 segundo: calcular L/min instantáneo y promedio parcial ===
  if (now - t1_sec >= 1000) {
    t1_sec += 1000;

    // Captura y resetea pulsos
    noInterrupts();
    uint16_t count = pulseCount;
    pulseCount = 0;
    interrupts();

    // Cálculo de caudal
    float L_per_s   = count / PULSES_PER_L;
    float L_per_min = L_per_s * 60.0;
    lastLpm = L_per_min;

    // Actualiza buffer y suma
    if (fullFlow) {
      sumFlow -= bufFlow[idxFlow];
    }
    bufFlow[idxFlow] = L_per_min;
    sumFlow += L_per_min;
    if (++idxFlow == 60) {
      idxFlow = 0;
      fullFlow = true;
    }

    // Calcula promedio parcial (hasta 60 muestras)
    uint8_t cnt = fullFlow ? 60 : idxFlow;
    float avgLpm = (cnt > 0) ? (sumFlow / cnt) : 0;

    // Mostrar en LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Inst:");
    lcd.print(L_per_min, 2);
    lcd.print(" L/m");
    lcd.setCursor(0, 1);
    lcd.print("Avg:");
    lcd.print(avgLpm, 2);
    lcd.print(" L/m");

    // Debug en Serial
    Serial.print("Inst L/min: ");
    Serial.print(L_per_min, 2);
    Serial.print("   Avg L/min: ");
    Serial.println(avgLpm, 2);
  }

  // === Cada 60 s: enviar promedio completo por RS-485 ===
  if (now - t60_sec >= 60000) {
    t60_sec += 60000;

    uint8_t cnt = fullFlow ? 60 : idxFlow;
    if (cnt == 0) return;
    float avgLpm = sumFlow / cnt;

    // Construir trama: i<ID>:<valor>f
    String msg = "i";
    msg += NODE_ID;
    msg += ":";
    msg += String(avgLpm, 2);
    msg += "f";

    // Transmisión RS-485
    digitalWrite(DE_RE_PIN, HIGH);
    delayMicroseconds(10);
    rs485.print(msg);
    rs485.flush();
    delayMicroseconds(10);
    digitalWrite(DE_RE_PIN, LOW);

    // Debug envío
    Serial.print("Enviado ");
    Serial.println(msg);
  }
}
