#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT Nano ===
#define ONE_WIRE_PIN 2     // DS18B20 DATA
#define DE_RE_PIN    8     // DE/RE del MAX485
#define RX_PIN      10     // RO del MAX485
#define TX_PIN      11     // DI del MAX485

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial rs485(RX_PIN, TX_PIN);

float buf[60];
uint8_t idx = 0;
bool full = false;
float sumT = 0;

unsigned long t1 = 0, t60 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);       // RX
  rs485.begin(9600);

  sensors.begin();

  lcd.init(); lcd.backlight();
  lcd.clear(); lcd.print("Iniciando...");
  delay(1000); lcd.clear();

  t1 = t60 = millis();
  for(int i=0;i<60;i++) buf[i]=0;
}

void loop(){
  unsigned long now = millis();

  // 1) Lectura 1 Hz
  if (now - t1 >= 1000) {
    t1 += 1000;
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    // LCD instantáneo
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T:"); lcd.print(tempC,2);
    lcd.write(0xDF); lcd.print("C");

    // Buffer para promedio
    if (full) sumT -= buf[idx];
    buf[idx] = tempC;
    sumT += tempC;
    if (++idx == 60) { idx=0; full=true; }
    Serial.print("Temp: "); Serial.println(tempC,2);
  }

  // 2) Promedio cada 60 s
  if (now - t60 >= 60000) {
    t60 += 60000;
    uint8_t cnt = full ? 60 : idx;
    if (cnt==0) return;
    float avg = sumT / cnt;

    // LCD promedio
    lcd.setCursor(0,1);
    lcd.print("P:"); lcd.print(avg,2);
    lcd.write(0xDF); lcd.print("C");

    // Envío RS-485: i107:XX.XXf
    digitalWrite(DE_RE_PIN, HIGH);  // TX
    rs485.print('i');
    rs485.print(107);
    rs485.print(':');
    rs485.print(avg,2);
    rs485.print('f');
    rs485.flush();
    digitalWrite(DE_RE_PIN, LOW);   // RX

    Serial.print("Enviado: i107:"); Serial.print(avg,2); Serial.println("f");
  }
}
