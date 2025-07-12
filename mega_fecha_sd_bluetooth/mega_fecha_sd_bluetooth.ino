/*  Id;Fecha;Hora;s1…s8  → SD + envío BT (HC-05)  */
#define SET_RTC_ONCE 0
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

/* ---- Pines ---- */
const uint8_t PIN_SD_CS = 53;      // CS lector SD (Mega)
const uint8_t BT_RX_PIN = 17;      // HC-05  TX → RX17
const uint8_t BT_TX_PIN = 16;      // Mega  TX16 → RX HC-05
const uint8_t sensorPins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};

/* ---- Objetos ---- */
LiquidCrystal_I2C lcd(0x27,16,2);
RTC_DS3231       rtc;
SoftwareSerial   bt(BT_RX_PIN, BT_TX_PIN);
File             todoFile;

/* ---- Variables ---- */
uint32_t rowId = 1;
const unsigned long T_MS = 1000UL;

/* ---------- utilidades ---------- */
void halt(const __FlashStringHelper* m){ lcd.clear(); lcd.print(m); while(1); }

void hdr(File& f){
  if (f.size()==0){
    f.println(F("Id;Fecha;Hora;s1;s2;s3;s4;s5;s6;s7;s8"));
    f.flush();
  }
}

void leerSensores(int16_t s[8]){ for(uint8_t i=0;i<8;i++) s[i]=analogRead(sensorPins[i]); }

void saveSample(const DateTime& t){
  int16_t s[8]; leerSensores(s);
  char line[120];
  sprintf(line,"%lu;%04u-%02u-%02u;%02u:%02u:%02u;%d;%d;%d;%d;%d;%d;%d;%d",
          rowId,t.year(),t.month(),t.day(),t.hour(),t.minute(),t.second(),
          s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7]);
  todoFile.println(line); todoFile.flush();
  Serial.println(line);   bt.println(line);
  rowId++;
}

/* --------------- SETUP --------------- */
void setup(){
  Serial.begin(9600);
  bt.begin(9600);

  lcd.init(); lcd.backlight(); lcd.print(F("Iniciando..."));
  if(!rtc.begin())       halt(F("Error RTC"));
  if(!SD.begin(PIN_SD_CS)) halt(F("Error SD"));

#if SET_RTC_ONCE
  rtc.adjust(DateTime(__DATE__,__TIME__));
#endif

  todoFile = SD.open("todo.csv", FILE_WRITE);
  if(!todoFile) halt(F("No CSV"));
  hdr(todoFile);

  for(uint8_t i=0;i<8;i++) pinMode(sensorPins[i], INPUT);

  lcd.clear(); lcd.print(F("Sistema listo"));
}

/* ---------------- LOOP ---------------- */
void loop(){
  static unsigned long t0 = 0;
  DateTime now = rtc.now();

  lcd.setCursor(0,0); lcd.print(now.timestamp(DateTime::TIMESTAMP_TIME));
  lcd.setCursor(0,1); lcd.print(now.timestamp(DateTime::TIMESTAMP_DATE));

  if(millis()-t0 >= T_MS){ t0 = millis(); saveSample(now); }

  /* -- comandos por USB o Bluetooth -- */
  if(Serial.available() || bt.available()){
    String cmd = Serial.available() ? Serial.readStringUntil('\n')
                                    : bt.readStringUntil('\n');
    cmd.trim();

    /* ---------- DESCARGA CSV ---------- */
    if(cmd.equalsIgnoreCase("SEND")){
      todoFile.flush();          // 1. vacía buffer
      todoFile.close();          // 2. cierra modo WRITE

      File f = SD.open("todo.csv", FILE_READ);   // 3. abre modo READ
      if(f){
        bt.println(F("---BEGIN CSV---"));
        while(f.available()) bt.write(f.read());
        f.close();
        bt.println(F("---END CSV---"));
      }else bt.println(F("ERROR: no se pudo abrir todo.csv"));

      todoFile = SD.open("todo.csv", FILE_WRITE); // 4. reabre para seguir
      todoFile.seek(todoFile.size());
    }

    /* ---------- AJUSTAR RELOJ ---------- */
    else if(cmd.startsWith("SET=")){
      int y,mo,d,h,mi,s;
      if(sscanf(cmd.c_str()+4,"%d-%d-%d,%d:%d:%d",&y,&mo,&d,&h,&mi,&s)==6){
        rtc.adjust(DateTime(y,mo,d,h,mi,s));
        Serial.println(F("RTC ajustado OK")); bt.println(F("RTC ajustado OK"));
      }else{
        Serial.println(F("Formato SET incorrecto")); bt.println(F("Formato SET incorrecto"));
      }
    }
  }
}