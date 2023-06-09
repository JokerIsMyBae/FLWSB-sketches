#include "SdsDustSensor.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include <rn2xx3.h>

#define DATA_LENGTH 20
#define SLEEPSECONDS 29

const char* appEui = "0000000000000000";
const char* appKey = "B123E59E88F18F43892A258D6D73B9FD";

/*

  | Byte nr | Name        | Sensor range         | On Node MCU | Reformat |
  | ------- | ----------- | -------------------- | ----------- | -------- |
  | 0       | Error byte  | n/a                  | n/a         | n/a      | n/a
  | 1-2     | Temperature | -40 tot 85°C         | +40 *10     | /100 -40 | - BME280
  | 3-5     | Pressure    | 300 tot 1100 hPa     | *100        | /100     | - BME280
  | 6-7     | Humidity    | 0 tot 100%           | *100        | /100     | - BME280
  | 8-9     | Temperature | -10 tot 60°C         | +10 *100    | /100 -10 | - SCD41
  | 10-11   | co2         | 400 tot 5000 ppm     | *100        | /100     | - SCD41
  | 12-13   | Humidity    | 0 tot 95 %           | *100        | /100     | - SCD41
  | 14-15   | PM2.5       | 0 tot 999 μg/m       | *10         | /10      | - SDS011
  | 16-17   | PM10        | 0 tot 999 μg/m       | *10         | /10      | - SDS011
  | 18-19   | Battery V   | 0 tot 3.3V           | *100        | /100     | n/a
  | 20-27   | Latitude    | -90 tot 90 (float)   |             |          | - GY-NEO6MV2
  | 28-35   | Longitude   | -180 tot 180 (float) |             |          | - GY-NEO6MV2
  
  If battery level equals 3.3V, this means it is between 3.3V and 4.2V

  error byte
  bit 4 = 1 -> GY-NEO not responding
  bit 3 = 1 -> battery lvl under 3.3V
  bit 2 = 1 -> SDS not responding
  bit 1 = 1 -> SCD not responding
  bit 0 = 1 -> BME not responding

*/

SensirionI2CScd4x scd4x;
Adafruit_BME280 bme;
SdsDustSensor sds(Serial1);

uint16_t co2_scd = 0, pm25_sds = 0, pm10_sds = 0;
float temp_bme = 0.0f, pres_bme = 0.0f, hum_bme = 0.0f, temp_scd = 0.0f,
      hum_scd = 0.0f, bat_lvl = 0.0f;
byte sensor_data[DATA_LENGTH];  // = 20; 19 bytes + one byte for error check
bool bme_status = true, scd_status = true, sds_status = true;

// LoRa variables
rn2xx3 myLora(Serial2);
double last_send_time = 0;

void setup() {
  Serial.begin(9600);   // serial port to computer
  Serial2.begin(9600);  // serial port to radio

  // Connected with BAT, analogRead() for bat lvl
  pinMode(PA02, INPUT);

  // PA03 = EN(able) of solar power manager; needs to be high
  pinMode(PA03, OUTPUT);
  digitalWrite(PA03, HIGH);

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

  Serial.println("Startup LoRa");

  initialize_radio();

  // transmit a startup message
  myLora.tx("TTN Mapper on TTN Enschede node");

  // Initialize rtc for sleep
  InitRTCInt();
}

void loop() {

  // measurements
  sensor_data[0] = 0;  // clear all errors
  executeMeasurements();

  formatData();

  // printing data (optional)
  for (int i = 0; i < DATA_LENGTH; i++) {
    Serial.println(sensor_data[i], HEX);
  }

  // send over LoRa
  Serial.println("TXing");
  double start = millis();
  
  // give data and data length; check declaration
  myLora.txBytes(sensor_data,DATA_LENGTH);
  double transmission = millis() - start;
  Serial.println(transmission);
  
  myLora.sleep(SLEEPSECONDS*1000);
  MCUsleep(SLEEPSECONDS);
  
  myLora.autobaud();
}

bool bmeTimeout(uint32_t& timeout_start) {
  timeout_start = millis();
  
  while (bme.isMeasuring()) {
    if ((millis() - timeout_start) > 2000)
      return false;
      
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
  MCUsleep(24);

  // Wake up scd now to get measurements at the same time as sds
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

    // if forced measurement has begun, time out until data is ready.
    if (bme_status)
      bme_status = bmeTimeout(timeout_start);
  }

  // if bme is in forced mode and data is ready, read data registers.
  if (bme_status) {
    temp_bme = bme.readTemperature();
    pres_bme = bme.readPressure() / 100.0F;
    hum_bme = bme.readHumidity();

    // if the data wasn't correct it returns NaN
    if (isnan(temp_bme) || isnan(pres_bme) || isnan(hum_bme))
      bme_status = false;
    
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

  // Check battery level
  bat_lvl = analogRead(PA02);
}

void formatData() {
  // If status of sensor is false, error has occured
  // Fill measurement bytes with error code and set corresponding bit in error
  // byte to 1
  // Else fill with measurements

  uint16_t bme_temp, bme_hum, scd_temp, scd_hum, scd_co2, sds_pm25, sds_pm10,
      lvl_bat;
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

  error_byte = !sds_status << 2 | !scd_status << 1 | !bme_status;
  if (bat_lvl < 1000)
    error_byte |= (1 << 3);

  bat_lvl = mapf(lvl_bat, 0, 1023, 0, 3.3);
  lvl_bat = bat_lvl * 100;

  sensor_data[0] = error_byte;

  sensor_data[1] = (bme_temp >> 8) & 0xFF;
  sensor_data[2] = bme_temp & 0xFF;
  sensor_data[3] = (bme_pres >> 16) & 0xFF;
  sensor_data[4] = (bme_pres >> 8) & 0xFF;
  sensor_data[5] = bme_pres & 0xFF;
  sensor_data[6] = (bme_hum >> 8) & 0xFF;
  sensor_data[7] = bme_hum & 0xFF;
  sensor_data[8] = (scd_temp >> 8) & 0xFF;
  sensor_data[9] = scd_temp & 0xFF;
  sensor_data[10] = (scd_co2 >> 8) & 0xFF;
  sensor_data[11] = scd_co2 & 0xFF;
  sensor_data[12] = (scd_hum >> 8) & 0xFF;
  sensor_data[13] = scd_hum & 0xFF;
  sensor_data[14] = (sds_pm25 >> 8) & 0xFF;
  sensor_data[15] = sds_pm25 & 0xFF;
  sensor_data[16] = (sds_pm10 >> 8) & 0xFF;
  sensor_data[17] = sds_pm10 & 0xFF;
  sensor_data[18] = (lvl_bat >> 8) & 0xFF;
  sensor_data[19] = lvl_bat & 0xFF;
  
}

void initialize_radio() {
  // reset rn2483
  Serial.println("resetting lora");
  pinMode(PA10, OUTPUT);
  digitalWrite(PA10, LOW);
  delay(1000);
  digitalWrite(PA10, HIGH);
  Serial.println("done resetting lora");

  // ingestelde appKey en joinEUI
  Serial.print("appKey: ");
  Serial.println(appKey);
  Serial.print("joinEUI: ");
  Serial.println(appEui);
  delay(100);  // wait for the RN2xx3's startup message
  Serial2.flush();

  // Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  // check communication with radio
  String hweui = myLora.hweui();
  while (hweui.length() != 16) {
    Serial.println(
        "Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  // print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(myLora.hweui());
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  // configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  join_result = myLora.initOTAA(appEui, appKey);

  // Loopt vast bij OTAA, uncomment voor ABP
  while (!join_result) {
    Serial.println(
        "Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(30000);  // delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
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

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
