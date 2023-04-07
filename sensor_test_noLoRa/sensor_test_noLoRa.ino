#include "SdsDustSensor.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>

#define DATA_LENGTH 18
#define SLEEPSECONDS 29

/*

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

  // Initialize rtc for sleep
  InitRTCInt();
}

void loop() {

  // measurements
  sensor_data[DATA_LENGTH - 1] = 0;  // clear all errors

  executeMeasurements();

  formatData();

  // printing data (optional)
  for (int i = 0; i < DATA_LENGTH; i++) {
    Serial.println(sensor_data[i], HEX);
  }

  MCUsleep(SLEEPSECONDS);
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

  // Sleep mcu for 25s
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

void InitRTCInt() {
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |    // 50-50 Duty(Though,it will not
                                            // reflect on any output pin)
                      GCLK_GENCTRL_GENEN |  // Enable generic clock generator
                      GCLK_GENCTRL_SRC_OSCULP32K |  // Internal 32kHz low
                                                    // power clock as source
                      GCLK_GENCTRL_ID(4) |          // Select GCLK 4
                      GCLK_GENCTRL_DIVSEL |   // Set GLCK divisor as 2 to the
                                              // power of (divisor) value
                      GCLK_GENCTRL_RUNSTDBY;  // Run on standby
  while (GCLK->STATUS.bit.SYNCBUSY) {};

  // Set Clock divider for GCLK4
  GCLK->GENDIV.reg =
      // Select clock divisor to divide by 32 (2 ^ (4 + 1))
      GCLK_GENDIV_DIV(4) |
      // GCLK4
      GCLK_GENDIV_ID(4);

  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY) {};

  // Connect GCLK4 output to RTC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK4 |  // Select GCLK4
                      GCLK_CLKCTRL_ID_RTC |     // Connect to the RTC
                      GCLK_CLKCTRL_CLKEN;       // Enable GCLK4
  // Wait for synchronization
  while (GCLK->STATUS.bit.SYNCBUSY) {};

  // -----------RTC configuration-----------
  // (rtc.h)
  RTC->MODE1.CTRL.bit.ENABLE = 0;  // Disable the RTC

  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};

  RTC->MODE1.CTRL.bit.SWRST = 1;  // Software reset the RTC

  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};

  RTC->MODE1.CTRL.reg |=
      RTC_MODE1_CTRL_PRESCALER_DIV1024 |  // Set prescaler to 1024
      RTC_MODE1_CTRL_MODE_COUNT16;        // Set RTC to mode 0, 32-bit timer

  // SET TIME IN SEC HERE AFTER RTC_MODE1_PER_PER
  RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(0);
  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};

  // Interrupt time in s: 1Hz/(#seconds + 1)
  RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(SLEEPSECONDS);
  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};

  // -----------Configure RTC interrupts-----------
  // Enable RTC overflow interrupt
  RTC->MODE1.INTENSET.reg = RTC_MODE1_INTENSET_OVF;

  // Set the Nested Vector Interrupt Controller (NVIC) priority for RTC
  NVIC_SetPriority(RTC_IRQn, 0);
  // Connect RTC to Nested Vector Interrupt Controller (NVIC)
  NVIC_EnableIRQ(RTC_IRQn);

  // -----------Enable Deep Sleep Mode-----------
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // | SCB_SCR_SLEEPONEXIT_Msk;  // Put
                                      // the SAMD21 in deep sleep upon
                                      // executing the __WFI() function

  // Disable auto power reduction during sleep - SAMD21 Errata 1.14.2
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_SLEEPPRM_DISABLED;
}

// ISR
void RTC_Handler() {
  RTC->MODE1.INTFLAG.bit.OVF = 1;  // Reset the overflow interrupt flag
}

void MCUsleep(int downtime) {
  RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(0);
  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};

  RTC->MODE1.PER.reg =
      RTC_MODE1_PER_PER(downtime);  // Interrupt time in s: 1Hz/(#seconds + 1)
  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};

  RTC->MODE1.CTRL.bit.ENABLE = 1;  // Enable the RTC
  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};

  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;  // Disable SysTick interrupts
  __DSB();  // Complete outstanding memory operations - not required for
            // SAMD21 ARM Cortex M0+
  __WFI();  // Put the SAMD21 into deep sleep, Zzzzzzzz...
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  // Enable SysTick interrupts

  // Disable the RTC  --> to stop count outside standby
  RTC->MODE1.CTRL.bit.ENABLE = 0;
  // Wait for synchronization
  while (RTC->MODE1.STATUS.bit.SYNCBUSY) {};
}