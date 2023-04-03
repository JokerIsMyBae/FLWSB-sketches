#include <SensirionI2CScd4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)

/*
  Vraag temp, pressure en humidity op van BME 
  en format als 5 bytes in een array meegegeven als parameter.

  | Byte nr | Name            | Sensor range     | On Node MCU | Reformat |
  | ------- | --------------- | ---------------- | ----------- | -------- |
  | 0-1     | Temperature     | -40 tot 85°C     | +40         | -40      | - BME280
  | 2-4     | Pressure        | 300 tot 1100 hPa | n/a         | n/a      | - BME280
  | 5-6     | Humidity        | 0 tot 100%       | n/a         | n/a      | - BME280
  | 7-8     | Temperature     | -10 tot 60°C     | +10         | -10      | - SCD41
  | 9-10    | co2             | 400 tot 5000 ppm | n/a         | n/a      | - SCD41
  | 11-12   | Humidity        | 0 tot 95 %       | n/a         | n/a      | - SCD41
*/

SensirionI2CScd4x scd4x;
Adafruit_BME280 bme;

uint16_t co2_scd = 0;
float temp_bme = 0.0f, pres_bme = 0.0f, hum_bme = 0.0f, temp_scd = 0.0f, hum_scd = 0.0f;
byte sensor_data[13];
unsigned status;

void setup() {
  Serial.begin(115200);
  while (!Serial) {};

  Wire.begin();
  
  scd4x.begin(Wire, 0x62);
  status = bme.begin(0x76, &Wire);
}

void loop() {
  measureSCD();
  if (!status) {
    temp_bme = 0xFFFF;
    pres_bme = 0xFFFFFF;
    hum_bme = 0xFFFF;
  } else {
    measureBME();
  }
  formatData();

  for (int i = 0; i < 13; i++) {
    Serial.println(sensor_data[i], HEX);
  }
}

void measureSCD() {
  bool isDataReady = false;
  uint16_t serialnr0, serialnr1, serialnr2, error;
  
  error = scd4x.getSerialNumber(serialnr0, serialnr1, serialnr2);
  if (error) {
    temp_scd = 0xFFFF;
    co2_scd = 0xFFFF;
    hum_scd = 0xFFFF;
    return;
  }
  error = scd4x.measureSingleShot();
  if (error) {
    temp_scd = 0xFFFF;
    co2_scd = 0xFFFF;
    hum_scd = 0xFFFF;
    return;
  }
  uint8_t i = 0;
  do {
    error = scd4x.getDataReadyFlag(isDataReady);
    if (error) {
      temp_scd = 0xFFFF;
      co2_scd = 0xFFFF;
      hum_scd = 0xFFFF;
      return;
    }
    i++;
  } while ( (!isDataReady) || (error && i < 5) );
  error = scd4x.readMeasurement(co2_scd, temp_scd, hum_scd);
  if (error) {
    temp_scd = 0xFFFF;
    co2_scd = 0xFFFF;
    hum_scd = 0xFFFF;
  }
}

void measureBME() {
  temp_bme = bme.readTemperature();
  pres_bme = bme.readPressure() / 100.0F;
  hum_bme = bme.readHumidity();
  if (isnan(temp_bme) || isnan(pres_bme) || isnan(hum_bme)) {
    temp_bme = 0xFFFF;
    pres_bme = 0xFFFF;
    hum_bme = 0xFFFF;
  }
}

void formatData() {
  uint16_t bme_temp = (temp_bme + 40) * 100;
  uint32_t bme_pres = pres_bme * 100;
  uint16_t bme_hum = hum_bme * 100;

  uint16_t scd_temp = (temp_scd + 10) * 100;
  uint16_t scd_hum = hum_scd * 100;
  uint16_t scd_co2 = co2_scd;

  sensor_data[0]  = (bme_temp >> 8) & 0xFF;
  sensor_data[1]  = bme_temp & 0xFF;
  sensor_data[2]  = (bme_pres >> 16) & 0xFF;
  sensor_data[3]  = (bme_pres >> 8) & 0xFF;
  sensor_data[4]  = bme_pres & 0xFF;
  sensor_data[5]  = (bme_hum >> 8) & 0xFF;
  sensor_data[6]  = bme_hum & 0xFF;
  sensor_data[7]  = (scd_temp >> 8) & 0xFF;
  sensor_data[8]  = scd_temp & 0xFF;
  sensor_data[9]  = (scd_co2 >> 8) & 0xFF;
  sensor_data[10] = scd_co2 & 0xFF;
  sensor_data[11] = (scd_hum >> 8) & 0xFF;
  sensor_data[12] = scd_hum & 0xFF;
}
