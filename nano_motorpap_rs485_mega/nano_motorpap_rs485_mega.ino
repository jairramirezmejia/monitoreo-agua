/*
  Dispensador de gotas configurable:
  - 5 gotas = 1800 micropasos (calibración previa)
  - Ahora indicas cuántas gotas quieres y el motor se detiene tras dispensarlas.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT Nano Motor SONCEBOZ (single-ended) ===
#define STEP_PIN      6    // CLK+
#define DIR_PIN       7    // CW+
#define ENABLE_PIN    3    // EN+ (activo LOW)

#define DE_RE_PIN     8    // DE/RE MAX485
#define RX_PIN       10   // RO MAX485
#define TX_PIN       11   // DI MAX485
#define NODE_ID     113   // ID en RS-485

#define STEPS_PER_REV   200   // 1.8°/paso
#define MICROSTEPS      16    // 1/16 step
#define TOTAL_STEPS_REV (STEPS_PER_REV * MICROSTEPS)

// 5 gotas = 1800 micropasos según calibración
const long STEPS_PER_5_DROPS = 1800;

// Cuántas gotas se desean dispensar:
int dropsToDispense = 7;  // <— ajusta este valor

// estado para que solo suceda una vez
bool done = false;

LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial    rs485(RX_PIN, TX_PIN);

long currentStep = 0;

// Intervalo (μs) por pulso a 10 RPM
unsigned long pulseIntervalUs(){
  return 60000000UL / (TOTAL_STEPS_REV * 10);
}

// Gira 'steps' micropasos en CW
void rotateFixedSteps(long steps){
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(ENABLE_PIN, LOW);
  unsigned long dt = pulseIntervalUs();
  for(long i = 0; i < steps; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(dt/2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(dt/2);
    currentStep++;
  }
  digitalWrite(ENABLE_PIN, HIGH);
}

// Envía la posición actual por RS-485
void sendPosition(){
  String msg = "i" + String(NODE_ID) + ":" + String(currentStep) + "f";
  digitalWrite(DE_RE_PIN, HIGH);
  delayMicroseconds(10);
  rs485.print(msg);
  rs485.flush();
  delayMicroseconds(10);
  digitalWrite(DE_RE_PIN, LOW);
}

// Muestra un mensaje en LCD y los pasos actuales
void displayLCD(const char* line1){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print("Steps:");
  lcd.print(currentStep);
}

void setup(){
  Serial.begin(9600);
  rs485.begin(9600);

  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);

  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Init Dispenser");
  delay(1000);
}

void loop(){
  if(!done){
    // Calcula micropasos para la cantidad de gotas deseada:
    // pasos = (STEPS_PER_5_DROPS * dropsToDispense) / 5
    long stepsNeeded = (STEPS_PER_5_DROPS * dropsToDispense) / 5;
    rotateFixedSteps(stepsNeeded);

    // Muestra y reporta
    char buf[16];
    sprintf(buf, "%d gotas", dropsToDispense);
    displayLCD(buf);
    sendPosition();

    done = true;  // no repetir
  }
  // se queda detenido aquí
}
