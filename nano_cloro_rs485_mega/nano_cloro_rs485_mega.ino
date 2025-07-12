#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === PINOUT ===
#define CL_RX_PIN     2   // TXD del ZW-RCl101 → RX del Nano
#define CL_TX_PIN     3   // RXD del ZW-RCl101 ← TX del Nano
#define DE_RE_PIN     8   // DE/RE del MAX485
#define RS485_RX_PIN 10   // RO del MAX485 → RX del Nano
#define RS485_TX_PIN 11   // DI del MAX485 ← TX del Nano
#define NODE_ID     108   // ID de este nodo en RS-485

LiquidCrystal_I2C lcd(0x27,16,2);
// Ahora sí tenemos RX _y_ TX para poder preguntar al sensor
SoftwareSerial sensorCl(CL_RX_PIN, CL_TX_PIN);
// Y nuestro bus RS-485
SoftwareSerial bus485(RS485_RX_PIN, RS485_TX_PIN);

// Buffer de 60 s para promediar
float bufCl[60];
uint8_t idxCl   = 0;
bool    fullCl  = false;
float   sumCl   = 0.0;
unsigned long t1  = 0;
unsigned long t60 = 0;

// Trama Modbus RTU: [Slave=1][Func=3][Addr=0x0040][Qty=2][CRClo][CRChi]
uint8_t modbusReq[8] = { 0x01, 0x03, 0x00, 0x40, 0x00, 0x02, 0, 0 };

// CRC16-IBM para Modbus RTU
uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

void setup() {
  Serial.begin(9600);

  // Completar el CRC de la petición
  uint16_t c = crc16(modbusReq, 6);
  modbusReq[6] = c & 0xFF;
  modbusReq[7] = c >> 8;

  // Arrancar UART TTL al módulo de cloro
  sensorCl.begin(9600);
  // Arrancar RS-485
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW);  
  bus485.begin(9600);

  // Arrancar LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cloro ID "); lcd.print(NODE_ID);
  delay(1000);
  lcd.clear();

  // Inicializar buffer y temporizadores
  for (int i = 0; i < 60; i++) bufCl[i] = 0.0;
  t1  = millis();
  t60 = millis();
}

void loop() {
  unsigned long now = millis();

  // === 1 Hz: pregunta y lee el sensor de cloro ===
  if (now - t1 >= 1000) {
    t1 += 1000;

    // Enviar petición Modbus
    sensorCl.write(modbusReq, sizeof(modbusReq));
    Serial.print(">> Peticion Modbus: ");
    for (uint8_t i = 0; i < 8; i++) { Serial.print(modbusReq[i], HEX); Serial.print(" "); }
    Serial.println();
    delay(50);

    // Si tenemos al menos 9 bytes, procesamos
    if (sensorCl.available() >= 9) {
      uint8_t rsp[9];
      for (int i = 0; i < 9; i++) rsp[i] = sensorCl.read();

      Serial.print("<< Respuesta: ");
      for (int i = 0; i < 9; i++) { Serial.print(rsp[i], HEX); Serial.print(" "); }
      Serial.println();

      // Validar slave y función
      if (rsp[0] == 0x01 && rsp[1] == 0x03 && rsp[2] == 0x04) {
        // Reconstruir float big-endian (bytes 3..6)
        union { uint8_t b[4]; float f; } u;
        u.b[3] = rsp[3];
        u.b[2] = rsp[4];
        u.b[1] = rsp[5];
        u.b[0] = rsp[6];
        float cloro = u.f;

        // Mostrar instantáneo en LCD línea 0
        lcd.setCursor(0,0);
        lcd.print("Inst Cloro:      ");
        lcd.setCursor(0,0);
        lcd.print("Inst Cloro:");
        lcd.print(cloro,2);
        lcd.print("mg/L");

        Serial.print("Inst Cloro = "); Serial.println(cloro,2);

        // Acumular para promedio
        if (fullCl) sumCl -= bufCl[idxCl];
        bufCl[idxCl] = cloro;
        sumCl       += cloro;
        if (++idxCl == 60) { idxCl = 0; fullCl = true; }
      }
    } else {
      Serial.println("!! No llegó respuesta del sensor");
    }
  }

  // === Cada 60 s: calcular promedio, mostrar y enviar ===
  if (now - t60 >= 60000) {
    t60 += 60000;
    uint8_t cnt = fullCl ? 60 : idxCl;
    if (cnt == 0) return;
    float avgCl = sumCl / cnt;

    // Mostrar promedio en LCD línea 1
    lcd.setCursor(0,1);
    lcd.print("Prom Cloro:      ");
    lcd.setCursor(0,1);
    lcd.print("Prom Cloro:");
    lcd.print(avgCl,2);
    lcd.print("mg/L");

    // Enviar via RS-485: i108:XX.XXf
    digitalWrite(DE_RE_PIN, HIGH);
    bus485.print('i'); bus485.print(NODE_ID);
    bus485.print(':'); bus485.print(avgCl,2); bus485.print('f');
    bus485.flush();
    digitalWrite(DE_RE_PIN, LOW);

    Serial.print("Enviado i108:"); Serial.println(avgCl,2);

    // Reiniciar buffer
    sumCl  = 0.0;
    idxCl  = 0;
    fullCl = false;
  }
}
