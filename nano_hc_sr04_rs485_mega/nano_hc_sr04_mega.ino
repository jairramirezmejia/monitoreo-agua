#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === CONFIGURACIÓN DE PINES ===
#define TRIG_PIN     2    // Pin digital para TRIG del HC-SR04
#define ECHO_PIN     3    // Pin digital para ECHO del HC-SR04
#define DE_RE_PIN    8    // DE/RE del MAX485
#define RX_PIN      10    // RO del MAX485 (RX del Nano)
#define TX_PIN      11    // DI del MAX485 (TX del Nano)

#define NODE_ID      8    // Dirección de este nodo en el bus RS-485

// === OBJETOS ===
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial rs485(RX_PIN, TX_PIN);

// === PARA CALCULAR PROMEDIO ===
float bufferDist[60];
uint8_t idx = 0;
bool full = false;
float sumDist = 0.0;

// === TIMERS ===
unsigned long lastSec = 0;
unsigned long lastMin = 0;

void setup() {
  Serial.begin(9600);

  // RS-485 DE/RE
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);    // modo RECEIVE
  rs485.begin(9600);

  // Sensor ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LCD I2C
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Iniciando...");
  delay(1000);
  lcd.clear();

  // Inicializar buffer y timers
  for (int i = 0; i < 60; i++) bufferDist[i] = 0;
  lastSec = lastMin = millis();
}

void loop() {
  unsigned long now = millis();

  // === 1) Medición cada segundo ===
  if (now - lastSec >= 1000) {
    lastSec += 1000;

    // Disparar pulso TRIG
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Leer duración ECHO
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms
    // Convertir a centímetros: velocidad sonido ≃ 343 m/s
    float dist = (duration / 2.0) * 0.0343; // cm

    // Mostrar lectura instantánea en LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Lvl:");
    lcd.print(dist,2);
    lcd.print("cm");

    Serial.print("Nivel instant: ");
    Serial.print(dist,2);
    Serial.println(" cm");

    // Actualizar buffer para promedio
    if (full) sumDist -= bufferDist[idx];
    bufferDist[idx] = dist;
    sumDist += dist;
    if (++idx == 60) { idx = 0; full = true; }
  }

  // === 2) Promedio cada 60 s ===
  if (now - lastMin >= 60000) {
    lastMin += 60000;
    uint8_t count = full ? 60 : idx;
    if (count == 0) return;

    float avg = sumDist / count;

    // Mostrar promedio en LCD en segunda línea
    lcd.setCursor(0,1);
    lcd.print("Prom:");
    lcd.print(avg,2);
    lcd.print("cm");

    Serial.print("Prom(60s): ");
    Serial.print(avg,2);
    Serial.println(" cm");

    // Enviar promedio por RS-485: i8:XX.XXf
    digitalWrite(DE_RE_PIN, HIGH);  // TX
    rs485.print('i');
    rs485.print(NODE_ID);
    rs485.print(':');
    rs485.print(avg, 2);
    rs485.print('f');
    rs485.flush();
    digitalWrite(DE_RE_PIN, LOW);   // RX

    Serial.print("Enviado RS485: i");
    Serial.print(NODE_ID);
    Serial.print(':');
    Serial.print(avg,2);
    Serial.println("f");
  }
}
