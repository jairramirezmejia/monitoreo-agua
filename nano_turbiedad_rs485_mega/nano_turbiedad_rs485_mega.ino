#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT Nano Turbidez ===
#define TURB_PIN     A0    // SEN0189: señal analógica
#define DE_RE_PIN     8    // DE/RE del MAX485
#define RX_PIN       10    // RO del MAX485
#define TX_PIN       11    // DI del MAX485
#define NODE_ID     106    // ID en la trama RS-485

LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial rs485(RX_PIN, TX_PIN);

// --- Parámetros de conversión (ajusta S tras ver los datos) ---
float V0;              // offset (se calibra en setup)
float S  = 187.5;      // sensibilidad NTU/V (valor inicial sugerido)

float buf[60];
uint8_t idx   = 0;
bool    full  = false;
float   sumTU = 0;

unsigned long t1  = 0;
unsigned long t60 = 0;

void setup(){
  Serial.begin(9600);

  // RS-485
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);    // RX
  rs485.begin(9600);

  // LCD
  lcd.init(); lcd.backlight();
  lcd.clear(); lcd.print("Calibrando V0");
  delay(1500);

  // 1) Calibrar offset V0 en agua de mesa
  const int N = 20;
  long sumRaw = 0;
  for(int i=0; i<N; i++){
    sumRaw += analogRead(TURB_PIN);
    delay(100);
  }
  float raw0 = sumRaw / float(N);
  V0 = raw0 * (5.0/1023.0);
  Serial.print("raw0 = "); Serial.println(raw0);
  Serial.print("V0   = "); Serial.println(V0,3);

  lcd.clear();
  lcd.print("V0="); lcd.print(V0,3);
  delay(2000);
  lcd.clear();

  // inicializar buffer y timers
  for(int i=0;i<60;i++) buf[i]=0;
  t1  = millis();
  t60 = millis();
}

void loop(){
  unsigned long now = millis();

  // === 1 Hz: lectura instantánea ===
  if(now - t1 >= 1000){
    t1 += 1000;
    int raw = analogRead(TURB_PIN);
    float V   = raw * (5.0/1023.0);
    float NTU = (V - V0) * S;
    if(NTU < 0) NTU = 0;  // sin clamp superior

    // Mostrar en LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Raw:"); lcd.print(raw);
    lcd.setCursor(0,1);
    lcd.print("V:");   lcd.print(V,3);
    lcd.print(" N:");  lcd.print(NTU,1);

    // Debug en Serial
    Serial.print("Raw="); Serial.print(raw);
    Serial.print("  V="); Serial.print(V,3);
    Serial.print("  NTU="); Serial.println(NTU,1);

    // Acumular para promedio
    if(full) sumTU -= buf[idx];
    buf[idx] = NTU;
    sumTU += NTU;
    if(++idx == 60){ idx=0; full=true; }
  }

  // === Cada 60 s: promedio y envío RS-485 ===
  if(now - t60 >= 60000){
    t60 += 60000;
    uint8_t cnt = full ? 60 : idx;
    if(cnt == 0) return;
    float avgTU = sumTU / cnt;

    // Mostrar promedio
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("P NTU:"); lcd.print(avgTU,1);

    // Enviar por RS-485
    digitalWrite(DE_RE_PIN, HIGH);   // TX
    rs485.print('i');
    rs485.print(NODE_ID);
    rs485.print(':');
    rs485.print(avgTU,1);
    rs485.print('f');
    rs485.flush();
    digitalWrite(DE_RE_PIN, LOW);    // RX

    Serial.print("Enviado i"); Serial.print(NODE_ID);
    Serial.print(':'); Serial.print(avgTU,1); Serial.println("f");
  }
}
