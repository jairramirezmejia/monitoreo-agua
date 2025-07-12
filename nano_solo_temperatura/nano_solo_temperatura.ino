//TEMPERATURA INICIAL NANO SOLO 

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT ===
#define ONE_WIRE_PIN 2    // Bus 1-Wire del DS18B20
#define DE_RE_PIN    8    // DE/RE del MAX485
#define RX_PIN      10    // RO del MAX485 (SoftSerial RX)
#define TX_PIN      11    // DI del MAX485 (SoftSerial TX)

// === Objetos ===
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial rs485(RX_PIN, TX_PIN);

// === Buffers para promedio ===
float bufferTemp[60];
uint8_t bufIndex    = 0;
bool    bufFull     = false;
float   sumTemps    = 0.0;

// === Temporizadores ===
unsigned long lastSecMillis = 0;
unsigned long lastMinMillis = 0;

void setup() {
  Serial.begin(9600);
  // RS-485
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);     // MODO RECEIVE
  rs485.begin(9600);

  // Sensor + LCD
  sensors.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Iniciando...");
  delay(1000);
  lcd.clear();

  // Init timers & buffer
  lastSecMillis = lastMinMillis = millis();
  for (int i = 0; i < 60; i++) bufferTemp[i] = 0.0;
}

void loop() {
  unsigned long now = millis();

  // —— 1. Lectura cada segundo ——
  if (now - lastSecMillis >= 1000UL) {
    lastSecMillis += 1000UL;
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    // Actualizar buffer
    if (bufFull) sumTemps -= bufferTemp[bufIndex];
    bufferTemp[bufIndex] = tempC;
    sumTemps += tempC;
    bufIndex = (bufIndex + 1) % 60;
    if (bufIndex == 0) bufFull = true;

    // Mostrar instantánea en LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T: ");
    lcd.print(tempC,2);
    lcd.write(0xDF); // °
    lcd.print("C");

    // Debug USB
    Serial.print("Temp: ");
    Serial.print(tempC,2);
    Serial.println(" C");
  }

  // —— 2. Promedio cada minuto ——
  if (now - lastMinMillis >= 60000UL) {
    lastMinMillis += 60000UL;
    uint8_t count = bufFull ? 60 : bufIndex;
    if (count == 0) return;

    float avg = sumTemps / count;

    // Mostrar promedio en LCD (línea 2)
    lcd.setCursor(0,1);
    lcd.print("P: ");
    lcd.print(avg,2);
    lcd.write(0xDF);
    lcd.print("C");

    // Envío por RS-485 en protocolo i<addr>:<valor>f\n
    digitalWrite(DE_RE_PIN, HIGH);   // TX
    rs485.print('i');
    rs485.print(107);                // ID nodo temperatura
    rs485.print(':');
    rs485.print(avg,2);
    rs485.print('f');
    rs485.print('\n');
    rs485.flush();
    digitalWrite(DE_RE_PIN, LOW);    // RX

    // Debug envío
    Serial.print("Enviado RS485: i107:");
    Serial.print(avg,2);
    Serial.println("f");
  }
}
