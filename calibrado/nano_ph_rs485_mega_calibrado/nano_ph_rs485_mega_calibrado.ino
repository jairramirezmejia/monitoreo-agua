#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT Nano pH en A0 ===
#define PH_PIN        A0  
#define DE_RE_PIN     8   
#define RS485_RX_PIN 10  
#define RS485_TX_PIN 11  
#define NODE_ID     101  

LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial   bus485(RS485_RX_PIN, RS485_TX_PIN);

// Buffer para promedio de 60 s
float bufPH[60];
uint8_t idxPH   = 0;
bool    fullPH  = false;
float   sumPH   = 0.0;
unsigned long t1 = 0, t60 = 0;

// --- PARÁMETROS CALIBRADOS ---
// Mide V7 y V4 en tus soluciones buffer y pon los valores aquí:
const float V7        = 2.50;           // Voltaje en pH = 7.00
const float V4        = 1.43;           // Voltaje en pH = 4.00
const float SLOPE_VpH = (V7 - V4) / 3.0; // ≈0.357 V/pH

void setup(){
  Serial.begin(9600);

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);
  bus485.begin(9600);

  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Nodo pH ID:"); lcd.print(NODE_ID);
  delay(1500); lcd.clear();

  for(int i=0;i<60;i++) bufPH[i]=0;
  t1 = millis(); t60 = millis();
}

void loop(){
  unsigned long now = millis();

  // 1 Hz: lectura instantánea
  if(now - t1 >= 1000){
    t1 += 1000;
    int raw = analogRead(PH_PIN);
    float V = raw * (5.0/1023.0);
    float pH = 7.0 + (V - V7)/SLOPE_VpH;
    pH = constrain(pH, 0.0, 14.0);

    // Línea 0: Inst
    lcd.setCursor(0,0);
    lcd.print("Inst pH:       ");
    lcd.setCursor(0,0);
    lcd.print("Inst pH:"); lcd.print(pH,2);

    Serial.print("Inst pH="); Serial.println(pH,2);

    // Buffer
    if(fullPH) sumPH -= bufPH[idxPH];
    bufPH[idxPH] = pH;
    sumPH += pH;
    if(++idxPH==60){ idxPH=0; fullPH=true; }
  }

  // Cada 60s: promedio y envío
  if(now - t60 >= 60000){
    t60 += 60000;
    uint8_t cnt = fullPH ? 60 : idxPH;
    if(cnt==0) return;
    float avgPH = sumPH / cnt;

    // Línea 1: Prom
    lcd.setCursor(0,1);
    lcd.print("Prom pH:       ");
    lcd.setCursor(0,1);
    lcd.print("Prom pH:"); lcd.print(avgPH,2);

    // Enviar i101:XX.XXf
    digitalWrite(DE_RE_PIN, HIGH);
    bus485.print('i'); bus485.print(NODE_ID);
    bus485.print(':'); bus485.print(avgPH,2); bus485.print('f');
    bus485.flush();
    digitalWrite(DE_RE_PIN, LOW);

    Serial.print("Enviado i"); Serial.print(NODE_ID);
    Serial.print(':'); Serial.println(avgPH,2);

    // Reset buffer
    sumPH = 0; idxPH = 0; fullPH = false;
  }
}
