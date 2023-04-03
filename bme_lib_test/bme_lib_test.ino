#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

Adafruit_BME280 bme;
float temp_bme = 0.0f, pres_bme = 0.0f, hum_bme = 0.0f;
unsigned status;

void setup() {
  Serial.begin(115200);
  while (!Serial) {};

  Wire.begin();
  status = bme.begin();

  bme.setSampling(
    Adafruit_BME280::sensor_mode::MODE_FORCED, 
    Adafruit_BME280::sensor_sampling::SAMPLING_X16, 
    Adafruit_BME280::sensor_sampling::SAMPLING_X16, 
    Adafruit_BME280::sensor_sampling::SAMPLING_X16, 
    Adafruit_BME280::sensor_filter::FILTER_X16);
}

void loop() {
  bme.takeForcedMeasurement();
  temp_bme = bme.readTemperature();
  pres_bme = bme.readPressure() / 100.0F;
  hum_bme = bme.readHumidity();

  Serial.print("Temp = ");
  Serial.println(temp_bme);
  Serial.print("Pres = ");
  Serial.println(pres_bme);
  Serial.print("Hum = ");
  Serial.println(hum_bme);  
}
