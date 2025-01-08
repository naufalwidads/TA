#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>
#include "RandomForest3.h"

Eloquent::ML::Port::RandomForest model;

#define BUF_SIZE 20
#define TIMEOUT  100
#define USE_CAST true

#define MOIST 0
#define TEMP  1
#define EC    2
#define PH    3

#define intervalLCD  3000
#define intervalLCD2  3000
#define intervalSensor  2000

BluetoothSerial SerialBT;


const byte ROPin = 5; // RX pin for ESP32 (GPIO16)
const byte DIPin = 4; // TX pin for ESP32 (GPIO17)
const byte DE = 16; // DE pin for RS485 (GPIO4)
const byte RE = 17; // RE pin for RS485 (GPIO5)

HardwareSerial mod(2); // Use Serial2

const byte dataMod[7][8] = {
  {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}, // moist
  {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA}, // temp
  {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xCA}, // ec
  {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0A}, // ph
};

/*Sama tetapi alamat 255 setiap sensor yang terhubung harus merespons, 
sensor akan mengganti 255 dengan alamat yang dikonfigurasi saat ini, 
ubah USE_CAST menjadi benar untuk menggunakan*/

const byte dataMod_B[7][8] = {
  {0xFF, 0x03, 0x00, 0x00, 0x00, 0x01, 0x91, 0xD4}, // moist
  {0xFF, 0x03, 0x00, 0x01, 0x00, 0x01, 0xC0, 0x14}, // temp
  {0xFF, 0x03, 0x00, 0x02, 0x00, 0x01, 0x30, 0x14}, // ec
  {0xFF, 0x03, 0x00, 0x03, 0x00, 0x01, 0x61, 0xD4}, // ph
};

// Incoming buffer
byte buf[BUF_SIZE];

LiquidCrystal_I2C lcd(0x27, 20, 4); // Alamat I2C(0x27) LCD dan Jenis LCD (20x4)
unsigned long sebelumSensor = 0;
unsigned long sebelumLCD = 0;
unsigned long sebelumLCD2 = 0;

void setup() {
  lcd.begin();  
  lcd.backlight();
  Serial.begin(115200);
  mod.begin(4800, SERIAL_8N1, ROPin, DIPin);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  lcd.setCursor((20-5)/2, 0);
  lcd.print("Hallo");
  lcd.setCursor((20-8)/2, 1);
  lcd.print("Pengguna");
  lcd.setCursor((20-14)/2, 2);
  lcd.print("Selamat Datang");
  delay(2000);
  lcd.clear();
  SerialBT.begin("ESP32_SoilMonitor"); 
  Serial.println("Ready..");
}

void loop() {
  uint16_t val;
  float Val1, Val2, Val4, Val5, RegPh, RegTemp, RegMoist;
  uint16_t val3, val6, val7, val8;

  val = GetModVal(MOIST);
  Val1 = val * 0.1;

  val = GetModVal(TEMP);
  Val2 = val * 0.1;

  val3 = GetModVal(EC);

  val = GetModVal(PH);
  Val4 = val * 0.1;

  // Regresi
  RegPh = 0.4204 * Val4 + 3.2936;
  RegTemp = 0.8145 * Val2 + 5.7951;
  RegMoist = 0.8908 * Val1 +2.0223;

  float soilTemperature = RegTemp;
  float soilMoisture = RegMoist;
  float soilPH = RegPh;
  float soilEC = val3;
  int lcdColumns = 20;
  String prediksi;

  //predict
  unsigned long sekarangSensor = millis();
  float CorpSample[4];
  CorpSample[0] = soilMoisture;
  CorpSample[1] = soilTemperature;
  CorpSample[2] = soilEC;
  CorpSample[3] = soilPH;

  Serial.println(model.predict(CorpSample));

  if(model.predict(CorpSample)==0){
    prediksi = "Cabai";
  }
  else if(model.predict(CorpSample)==1){
    prediksi = "Jagung";
  }
  else if(model.predict(CorpSample)==2){
    prediksi = "Jeruk";
  }
  else if(model.predict(CorpSample)==3){
    prediksi = "Pakcoi";
  }
  else if(model.predict(CorpSample)==4){
    prediksi = "Rosemari";
  }
  else if(model.predict(CorpSample)==5){
    prediksi = "Selada";
  }
  else if(model.predict(CorpSample)==6){
    prediksi = "Tomat";
  }

  int Justify = (lcdColumns - prediksi.length()) / 2; //rata tengah
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(soilTemperature);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("humidity:");
    lcd.print(soilMoisture);
    lcd.print("%");

    lcd.setCursor(0, 2);
    lcd.print("pH:");
    lcd.print(soilPH);

    lcd.setCursor(0, 3);
    lcd.print("EC:");
    lcd.print(soilEC);
    unsigned long sekarangLCD2 = millis();
    if(CorpSample[2]>0){
        String data = String(prediksi) + "," + 
                  String(soilTemperature) + "," +
                  String(soilMoisture) + "," +
                  String(soilPH) + "," +
                  String(soilEC);
    // Send the data over Bluetooth
    SerialBT.println(data);

    // Print the data to the serial monitor for debugging
    Serial.println(data);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(soilTemperature);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("humidity:");
    lcd.print(soilMoisture);
    lcd.print("%");

    lcd.setCursor(0, 2);
    lcd.print("pH:");
    lcd.print(soilPH);

    lcd.setCursor(0, 3);
    lcd.print("EC:");
    lcd.print(soilEC);  
      if (sekarangLCD2 - sebelumLCD2 >= intervalLCD2){
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Rekomendasi Tanaman");
      lcd.setCursor(Justify, 2);
      lcd.print(prediksi);
       sebelumLCD2 = sekarangLCD2;
      }
    }
    
    else{
          String data = String("Silahkan Gunakan Sensor") + "," + 
                  String(soilTemperature) + "," +
                  String(soilMoisture) + "," +
                  String(soilPH) + "," +
                  String(soilEC);
    SerialBT.println(data);
    Serial.println(data);
      if (sekarangLCD2 - sebelumLCD2 >= intervalLCD2){
        lcd.clear();
        lcd.setCursor(2, 1);
        lcd.print("Silahkan Gunakan");
        lcd.setCursor(7, 2);
        lcd.print("Sensor");
      sebelumLCD2 = sekarangLCD2;
      }
    }
  delay(1000);
}

uint16_t GetModVal(byte val) {
  uint32_t startTime = 0;
  uint8_t byteCount = 0;
  memset(buf, 0, sizeof(buf)); // Empty incoming buffer
  // Send request
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);
  if (USE_CAST)
    mod.write(dataMod_B[val], sizeof(dataMod_B[val]));
  else
    mod.write(dataMod[val], sizeof(dataMod[val]));
  mod.flush(); // Wait for outgoing to be sent
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  // Receive response until timeout expires
  startTime = millis();
  while (millis() - startTime <= TIMEOUT) {
    if (mod.available() && byteCount < sizeof(buf)) {
      buf[byteCount++] = mod.read();
    }
  }
  // Combine 2 bytes into word
  return (uint16_t)(buf[3] << 8 | buf[4]);
}


