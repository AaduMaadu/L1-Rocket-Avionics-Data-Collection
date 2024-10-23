#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

/*
CS = 5
MOSI = 23
MISO 19
SCK 18
*/

#define LED 2

#define SEA_LVL_PRESSURE 1013.25 //hectopascal
Adafruit_BMP280 bmp; //I2C Comm Protocol

File myFile; //Define myFile obj
const int CS = 5; //Set the GOIP pin number for the chip select

void WriteFile(const char* path) {
  myFile = SD.open(path, FILE_WRITE); //Can only open one file at a time

  if (myFile) {
    Serial.printf("Writing to %s", path);
    myFile.print("Temperature = ");
    myFile.print(1.8 * bmp.readTemperature() + 32);
    myFile.println(" *F");
    myFile.close();
    Serial.println("completed");
  }
  else {
    Serial.println("Error opening the file");
    Serial.println(path);
  }
}

void ReadFile(const char* path) {
  myFile = SD.open(path, FILE_READ);

  if (myFile) {
    Serial.printf("Reading from %s", path);
    while (myFile.available()) {
      Serial.print((char)myFile.read());
    }
    myFile.close();
  }
  else {
    Serial.println("Error opening the file");
    Serial.println(path);
  }
}

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(LED, OUTPUT);

  while (!Serial) { ; } //Wait for serial port to connect. Needed for native USB port only

  Serial.println("Initializing SD card...");
  if (!SD.begin(CS)) {
    Serial.println("Intialization failed!");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initalization done");

  bool status = bmp.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 Sensor! Data Collection Halted.");
  }
  else {
    Serial.println("BMP 280 Found. Collecting Data...");
  }

  WriteFile("/test.txt");
  ReadFile("/test.txt");
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(5000);
  digitalWrite(LED, LOW);
  delay(250);
  digitalWrite(LED, HIGH);
  delay(250);
  digitalWrite(LED, LOW);
  delay(250);

  //Read Altitude
  Serial.print(bmp.readAltitude(SEA_LVL_PRESSURE));
  Serial.println(" m");
  delay (100);
  //Read Pressure
  Serial.print(bmp.readPressure() / 100.0F);
  Serial.println(" hPa");

}

// bmp.readTemperature();
  // bmp.readPressure();
  // bmp.readAltitude(seaLevelPressure);
  // SD.begin(chipSelect);
  // SD.open(filename, mode);
  // file.write();
  // file.read();