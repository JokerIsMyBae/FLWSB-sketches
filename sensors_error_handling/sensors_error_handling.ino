#include "SdsDustSensor.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>

#define DATA_LENGTH 18

/*
  Vraag temp, pressure en humidity op van BME
  en format als 5 bytes in een array meegegeven als parameter.
  | Byte nr | Name        | Sensor range     | On Node MCU | Reformat |
  | ------- | ----------- | ---------------- | ----------- | -------- |
  | 0-1     | Temperature | -40 tot 85°C     | +40 *10     | /100 -40 | - BME280
  | 2-4     | Pressure    | 300 tot 1100 hPa | *100        | /100     | - BME280
  | 5-6     | Humidity    | 0 tot 100%       | *100        | /100     | - BME280
  | 7-8     | Temperature | -10 tot 60°C     | +10 *100    | /100 -10 | - SCD41
  | 9-10    | co2         | 400 tot 5000 ppm | *100        | /100     | - SCD41
  | 11-12   | Humidity    | 0 tot 95 %       | *100        | /100     | - SCD41
  | 13-14   | PM2.5       | 0 tot 999 μg/m   | *10         | /10      | - SDS011
  | 15-16   | PM10        | 0 tot 999 μg/m   | *10         | /10      | - SDS011

  error byte
  bit 6 = 1 -> SDS not responding
  bit 3 = 1 -> SCD not responding
  bit 0 = 1 -> BME not responding
*/

SensirionI2CScd4x scd4x;
Adafruit_BME280 bme;
SdsDustSensor sds(Serial1);

uint16_t co2_scd = 0, pm25_sds = 0, pm10_sds = 0;
float temp_bme = 0.0f, pres_bme = 0.0f, hum_bme = 0.0f, temp_scd = 0.0f,
      hum_scd = 0.0f;
byte sensor_data[DATA_LENGTH];  // = 18; 17 bytes + one byte for error check
bool bme_status = true, scd_status = true, sds_status = true;

void setup() {
  Serial.begin(9600);  // serial port to computer

  Wire.begin();

  // Initialize sensors
  scd4x.begin(Wire);
  bme_status = bme.begin();
  sds.begin();
  sds.setQueryReportingMode();

  if (bme_status) {
    delay(10);  // bme startup time
    while (bme.isReadingCalibration()) {};
    bme.readCoefficients();
    bme.setSampling(Adafruit_BME280::sensor_mode::MODE_FORCED);
  }
}

void loop() {
  // measurements
  sensor_data[DATA_LENGTH - 1] = 0x00;  // clear all errors

  executeMeasurements();

  formatData();

  // printing data (optional)
  for (int i = 0; i < DATA_LENGTH; i++) {
    Serial.println(sensor_data[i], HEX);
  }

  delay(30000);
}

bool bmeTimeout(uint32_t& timeout_start) {
  timeout_start = millis();
  while (bme.isMeasuring()) {
    if ((millis() - timeout_start) > 2000) {
      return false;
    }
    delay(1);
  }
  return true;
}

void executeMeasurements() {
  uint16_t serialnr0, serialnr1, serialnr2, error;
  uint8_t i = 0;
  uint32_t timeout_start;

  // Wake-up sequence sensors
  sds.wakeup();

  // wait 25s out of 30s for sds measurement
  delay(25000);

  // Wake up scd to get measurements at the same time as sds
  error = scd4x.wakeUp();
  if (error)
    scd_status = false;

  // wake-up time for scd41
  if (scd_status)
    delay(20);

  // check if sensor woke up
  if (scd_status)
    error = scd4x.getSerialNumber(serialnr0, serialnr1, serialnr2);
  if (error)
    scd_status = false;

  // Send out start measurement commands
  if (scd_status)
    error = scd4x.measureSingleShot();
  if (error)
    scd_status = false;

  // If status == false, then bme hasn't been properly initialised
  if (bme_status) {
    bme_status = bme.takeForcedMeasurement();

    // if forced measurement has started, time out until data is ready.
    if (bme_status)
      bme_status = bmeTimeout(timeout_start);
  }

  // if bme is in forced mode and data is ready, read data registers.
  if (bme_status) {
    temp_bme = bme.readTemperature();
    pres_bme = bme.readPressure() / 100.0F;
    hum_bme = bme.readHumidity();

    // if the data wasn't correct it returns NaN
    if (isnan(temp_bme) || isnan(pres_bme) || isnan(hum_bme)) {
      bme_status = false;
    }
  }

  // measureSingleShot needs delay of 5000ms to wait for measurements
  // set to sleep for the remaining time of 5s
  delay(5000 - (millis() - timeout_start));

  // check if data ready
  if (scd_status)
    error = scd4x.getDataReadyFlag(scd_status);
  if (error)
    scd_status = false;

  // read data
  if (scd_status)
    error = scd4x.readMeasurement(co2_scd, temp_scd, hum_scd);
  if (error)
    scd_status = false;

  // request measurement results from sds
  PmResult pm = sds.queryPm();
  if (!pm.isOk()) {
    sds_status = false;
  } else {
    pm25_sds = pm.pm25;
    pm10_sds = pm.pm10;
  }

  // turn off sensors
  scd4x.powerDown();
  sds.sleep();
}

void formatData() {

  // If status of sensor is false, error has occured
  // Fill measurement bytes with error code and set corresponding bit in error
  // byte to 1
  // Else fill with measurements

  uint16_t bme_temp, bme_hum, scd_temp, scd_hum, scd_co2, sds_pm25, sds_pm10;
  uint32_t bme_pres;
  byte error_byte = 0;

  if (bme_status) {
    bme_temp = (temp_bme + 40) * 100;
    bme_pres = pres_bme * 100;
    bme_hum = hum_bme * 100;
  } else {
    bme_temp = 0xFFFF;
    bme_pres = 0xFFFFFF;
    bme_hum = 0xFFFF;
  }

  if (scd_status) {
    scd_temp = (temp_scd + 10) * 100;
    scd_hum = hum_scd * 100;
    scd_co2 = co2_scd;
  } else {
    scd_temp = 0xFFFF;
    scd_hum = 0xFFFF;
    scd_co2 = 0xFFFF;
  }

  if (sds_status) {
    sds_pm25 = pm25_sds * 10;
    sds_pm10 = pm10_sds * 10;
  } else {
    sds_pm25 = 0xFFFF;
    sds_pm10 = 0xFFFF;
  }

  error_byte = !sds_status << 6 | !scd_status << 3 | !bme_status;

  sensor_data[0] = (bme_temp >> 8) & 0xFF;
  sensor_data[1] = bme_temp & 0xFF;
  sensor_data[2] = (bme_pres >> 16) & 0xFF;
  sensor_data[3] = (bme_pres >> 8) & 0xFF;
  sensor_data[4] = bme_pres & 0xFF;
  sensor_data[5] = (bme_hum >> 8) & 0xFF;
  sensor_data[6] = bme_hum & 0xFF;
  sensor_data[7] = (scd_temp >> 8) & 0xFF;
  sensor_data[8] = scd_temp & 0xFF;
  sensor_data[9] = (scd_co2 >> 8) & 0xFF;
  sensor_data[10] = scd_co2 & 0xFF;
  sensor_data[11] = (scd_hum >> 8) & 0xFF;
  sensor_data[12] = scd_hum & 0xFF;
  sensor_data[13] = (sds_pm25 >> 8) & 0xFF;
  sensor_data[14] = sds_pm25 & 0xFF;
  sensor_data[15] = (sds_pm10 >> 8) & 0xFF;
  sensor_data[16] = sds_pm10 & 0xFF;
  sensor_data[17] = error_byte;
}
