#include <SensirionI2CScd4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)

SensirionI2CScd4x scd41;
Adafruit_BME280 bme;

bool isDataReady = false;
uint16_t serialnr0, serialnr1, serialnr2, error;
char errorMessage[256];
uint16_t co2 = 0;
float temperature = 0.0f, humidity = 0.0f;

void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  
  Wire.begin();
  
  scd41.begin(Wire, 0x62);
  unsigned status;
  status = bme.begin(0x76, &Wire);
  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  
}

void loop() {
  delay(1000);
  error = scd41.getSerialNumber(serialnr0, serialnr1, serialnr2);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    delay(20);
    return; // return to top of void loop()
  }
  uint8_t i = 0;
  do {
    Serial.println("Waiting for first measurement... (5 sec)");
    error = scd41.measureSingleShot();
    if (error) {
      Serial.print("Error trying to execute measureSingleShot(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
    i++;
  } while (error && i < 5);
  
  if (i >= 5) { return; }

  i = 0;
  do {
    error = scd41.getDataReadyFlag(isDataReady);
    if (error) {
      Serial.print("Error trying to execute getDataReadyFlag(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);  
    }
    i++;
  } while (error && i < 5);

  if (i >= 5) { return; };

  error = scd41.readMeasurement(co2, temperature, humidity);
  if (error) {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (co2 == 0) {
    Serial.println("Invalid sample detected, skipping.");
  } else {
    Serial.print("Co2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.println(humidity);
  }

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
    
}
