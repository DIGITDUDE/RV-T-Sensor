#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <OLED_I2C.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_SPIDevice.h>

#define DHTTYPE DHT11
#define DHTPIN 2

#define OLEDTYPE SSD1306_128X64
// put function declarations here:
OLED MyOLED(SDA,SCL);
extern uint8_t SmallFont[];

DHT_Unified MyDHT(DHTPIN, DHTTYPE);
unsigned long sensordelay;
unsigned long lastSensorCheck;

Adafruit_BMP085 bmp;

void setup() {
  // put your setup code here, to run once:
  MyOLED.begin(OLEDTYPE);
  MyDHT.begin();
  bmp.begin();
  MyOLED.setFont(SmallFont);
  if (!bmp.begin()) {
	MyOLED.print(F("no BMP085 sensor"),0,20);
	while (1) {}
  }
  
  
    // getting and writing Sensor data to Oled
  sensor_t sensor;
  MyOLED.clrScr();
  MyDHT.temperature().getSensor(&sensor);
  MyOLED.print(F("DHT 11 T sensor"),0,5);
  MyOLED.print(F("Driver Ver :"),0,20); MyOLED.printNumI(sensor.version,80,20);
  MyOLED.print(F("Max Value  :"),0,30); MyOLED.printNumI(sensor.max_value,80,30);
  MyOLED.print(F("Min value  :"),0,40); MyOLED.printNumI(sensor.min_value,80,40);
  MyOLED.print(F("resolution :"),0,50); MyOLED.printNumI(sensor.resolution,80,50);
  MyOLED.update();
  delay(5000);
  MyOLED.clrScr();
  MyDHT.humidity().getSensor(&sensor);
  MyOLED.print(F("DHT11 RV sensor"),0,5);
  MyOLED.print(F("Driver Ver :"),0,20); MyOLED.printNumI(sensor.version,80,20);
  MyOLED.print(F("Max Value  :"),0,30); MyOLED.printNumI(sensor.max_value,80,30);
  MyOLED.print(F("Min value  :"),0,40); MyOLED.printNumI(sensor.min_value,80,40);
  MyOLED.print(F("resolution :"),0,50); MyOLED.printNumI(sensor.resolution,80,50);
  MyOLED.update();
  delay(5000);
  sensordelay = sensor.min_delay / 1000;
  
}

void loop() {
  // put your main code here, to run repeatedly:
millis();

if ((millis() - lastSensorCheck) > sensordelay)
{
  MyOLED.clrScr();
  sensors_event_t event;
  MyDHT.temperature().getEvent(&event);
  MyOLED.print(F("temperature and RV"),0,5);
  MyOLED.print(F("temperature:"),0,20); MyOLED.printNumI(event.temperature,80,20);MyOLED.print(F("*C"),100,20);
  MyDHT.humidity().getEvent(&event);
  MyOLED.print(F("RV         :"),0,30); MyOLED.printNumI(event.relative_humidity,80,30);MyOLED.print(F("%"),100,30);
  MyOLED.print(F("BMP T      :"),0,40); MyOLED.printNumI(bmp.readTemperature(),80,40);MyOLED.print(F("*C"),100,40);
  MyOLED.print(F("Alltitude  :"),0,50); MyOLED.printNumI(bmp.readAltitude(),80,50);MyOLED.print(F("Mtr"),100,50);
  MyOLED.update();
  lastSensorCheck = millis();
}         

// put function definitions here:
}
 

