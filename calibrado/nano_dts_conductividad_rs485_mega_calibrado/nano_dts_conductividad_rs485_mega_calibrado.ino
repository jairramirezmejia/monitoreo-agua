#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT Nano TDS ===
#define TDS_PIN      A0    // Salida analógica del sensor TDS
#define DE_RE_PIN     8    // DE/RE del MAX485
#define RX_PIN       10    // RO del MAX485
#define TX_PIN       11    // DI del MAX485
#define NODE_ID     104    // ID de este nodo en el bus RS-485

LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial    rs485(RX_PIN, TX_PIN);

// Buffer para promedio 60 s
float bufferTDS[60];
uint8_t idx      = 0;
bool    full     = false;
float   sumTDS   = 0.0;
unsigned long t1 = 0, t60 = 0;

// Constantes ADC
const float VREF    = 5.0;    // Voltaje de referencia Arduino
const float ADC_MAX = 1023.0; // ADC 10 bit

// Offset “seco” (raw y voltaje) que medimos en aire
int   rawDryCount = 20; 
float rawDry = 0, Vdry = 0;

// Polinomio DFRobot para TDS (ppm)
float tdsFromVoltage(float V) {
  return 133.42*V*V*V
       - 255.86*V*V
       + 857.39*V;
}

void setup(){
  Serial.begin(9600);

  // RS-485
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);
  rs485.begin(9600);

  // LCD
  lcd.init(); lcd.backlight();
  lcd.clear();
  lcd.print("Calibrar seco");
  delay(2000);

  // 1) Medir rawDry en aire (raw y voltaje)
  long sumRaw = 0;
  for(int i=0;i<rawDryCount;i++){
    sumRaw += analogRead(TDS_PIN);
    delay(100);
  }
  rawDry = sumRaw / float(rawDryCount);
  Vdry   = rawDry * (VREF/ADC_MAX);

  Serial.print("rawDry="); Serial.println(rawDry);
  Serial.print("Vdry=");   Serial.println(Vdry,3);

  lcd.clear();
  lcd.print("Vdry="); lcd.print(Vdry,3);
  delay(2000);
  lcd.clear();

  // Inicializar buffer y timers
  for(int i=0;i<60;i++) bufferTDS[i] = 0.0;
  t1  = millis();
  t60 = millis();
}

void loop(){
  unsigned long now = millis();

  // --- 1 Hz: lectura instantánea ---
  if(now - t1 >= 1000){
    t1 += 1000;
    // 1) Leer ADC y voltaje
    int raw = analogRead(TDS_PIN);
    float V   = raw * (VREF/ADC_MAX);

    // 2) Restar offset “seco”
    float Vsig = V - Vdry;
    if(Vsig < 0) Vsig = 0;

    // 3) Calcular TDS solo si Vsig>0
    float tds = tdsFromVoltage(Vsig);
    if(tds < 0) tds = 0;

    // Grabar en buffer
    if(full) sumTDS -= bufferTDS[idx];
    bufferTDS[idx] = tds;
    sumTDS += tds;
    if(++idx == 60){ idx=0; full=true; }

    // Mostrar instantáneo
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Vsig:"); lcd.print(Vsig,2);
    lcd.setCursor(0,1);
    lcd.print("TDS: "); lcd.print(tds,1);

    // Debug USB
    Serial.print("raw="); Serial.print(raw);
    Serial.print(" V="); Serial.print(V,3);
    Serial.print(" Vsig="); Serial.print(Vsig,3);
    Serial.print(" TDS="); Serial.println(tds,1);
  }

  // --- Cada 60 s: promedio y envío ---
  if(now - t60 >= 60000){
    t60 += 60000;
    uint8_t count = full ? 60 : idx;
    if(count == 0) return;
    float avgTDS = sumTDS / count;

    // Mostrar promedio
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("P TDS:"); lcd.print(avgTDS,1); lcd.print("ppm");

    // Enviar RS-485
    digitalWrite(DE_RE_PIN, HIGH);
    rs485.print('i'); rs485.print(NODE_ID);
    rs485.print(':'); rs485.print(avgTDS,1); rs485.print('f');
    rs485.flush();
    digitalWrite(DE_RE_PIN, LOW);

    // Reset buffer para el siguiente minuto
    sumTDS = 0;
    idx    = 0;
    full   = false;
  }
}
