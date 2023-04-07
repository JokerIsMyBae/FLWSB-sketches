#include "SdsDustSensor.h"
// #include <Adafruit_BME280.h>
// #include <Adafruit_Sensor.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include <rn2xx3.h>

#define DATA_LENGTH 18

int sleepseconds = 119;

const char* appEui = "0000000000000000";
const char* appKey = "5E7773DF01C66243843429D3B38C5FCB";

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
  bit 7 = 1 -> BME not responding
  bit 6 = 1 -> BME temp error
  bit 5 = 1 -> BME pressure error
  bit 4 = 1 -> BME humidity error
  bit 3 = 1 -> SCD not responding
  bit 2 = 1 -> SCD temprature error
  bit 1 = 1 -> SCD co2 error
  bit 0 = 1 -> SCD humidity error

*/

SensirionI2CScd4x scd4x;
// Adafruit_BME280 bme;
SdsDustSensor sds(Serial1);

uint16_t co2_scd = 0, pm25_sds = 0, pm10_sds = 0;
float temp_bme = 0.0f, pres_bme = 0.0f, hum_bme = 0.0f, temp_scd = 0.0f,
      hum_scd = 0.0f;
byte sensor_data[DATA_LENGTH];  // = 18; 17 bytes + one byte for error check
unsigned status;

// LoRa variables
rn2xx3 myLora(Serial2);
double last_send_time = 0;

void setup() {
    Serial.begin(57600);   // serial port to computer
    Serial2.begin(57600);  // serial port to radio

    Wire.begin();

    // Initialize sensors
    scd4x.begin(Wire);
    // status = bme.begin();
    sds.setQueryReportingMode();

    // delay(10);  // bme startup time

    // while (bme.isReadingCalibration())
    // delay(1);
    // bme.readCoefficients();

    // if (status)
    // bme.setSampling(Adafruit_BME280::sensor_mode::MODE_FORCED);

    Serial.println("Startup LoRa");

    initialize_radio();

    // transmit a startup message
    myLora.tx("TTN Mapper on TTN Enschede node");

    // Initialize rtc for sleep
    InitRTCInt();
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

    // send over LoRa
    Serial.println("TXing");
    double start = millis();

    myLora.txBytes(sensor_data,
                   14);  // give data and data length; check declaration
    double transmission = millis() - start;
    Serial.println(transmission);

    MCUsleep(sleepseconds);
}

// bool bmeTimeout(uint32_t& timeout_start) {
//     timeout_start = millis();
//     while (bme.isMeasuring()) {  // read8 is private, provide interface?
//         if ((millis() - timeout_start) > 2000) {
//             return false;
//         }
//         delay(1);
//     }
//     return true;
// }

void executeMeasurements() {
    bool isDataReady = false, correctMode = false;
    uint16_t serialnr0, serialnr1, serialnr2, error;
    uint8_t i = 0;
    // uint32_t timeout_start;

    // Wake-up sequence sensors
    sds.wakeup();

    // Sleep mcu for 25s
    MCUsleep(24);

    // Wake up scd to get measurements at the same time as sds
    error = scd4x.wakeUp();

    // wake-up time for scd41
    delay(20);

    // check if sensor woke up
    error = scd4x.getSerialNumber(serialnr0, serialnr1, serialnr2);

    // Send out start measurement commands
    error = scd4x.measureSingleShot();

    // If status == false, then bme hasn't been properly initialised
    // if (!status) {
    temp_bme = 0xFFFF;
    pres_bme = 0xFFFFFF;
    hum_bme = 0xFFFF;
    // sensor_data[13] |= (1 << 7);  // error setting up BME
    // } else {
    // correctMode = bme.takeForcedMeasurement();

    // if forced measurement has started, time out until data is ready.
    // if (correctMode)
    // isDataReady = bmeTimeout(timeout_start);
    // }

    // if bme is in forced mode and data is ready, read data registers.
    // if (correctMode && isDataReady) {
    //     temp_bme = bme.readTemperature();
    //     pres_bme = bme.readPressure() / 100.0F;
    //     hum_bme = bme.readHumidity();

    //     // if the data wasn't correct it returns NaN
    //     if (isnan(temp_bme) || isnan(pres_bme) || isnan(hum_bme)) {
    //         temp_bme = 0xFFFF;
    //         pres_bme = 0xFFFFFF;
    //         hum_bme = 0xFFFF;
    //         sensor_data[DATA_LENGTH - 1] |= (111 << 4);
    //     }
    // }

    // measureSingleShot needs delay of 5000ms to wait for measurements
    // set to sleep for the remaining time of 5s
    delay(5000);

    // check if data ready
    error = scd4x.getDataReadyFlag(isDataReady);

    // read data
    error = scd4x.readMeasurement(co2_scd, temp_scd, hum_scd);

    // request measurement results from sds
    PmResult pm = sds.queryPm();
    if (!pm.isOk()) {
        pm25_sds = 0xFFFF;
        pm10_sds = 0xFFFF;
    }
    pm25_sds = pm.pm25;
    pm10_sds = pm.pm10;

    // turn off sensors
    scd4x.powerDown();
    sds.sleep();
}

void formatData() {
    uint16_t bme_temp = (temp_bme + 40) * 100;
    uint32_t bme_pres = pres_bme * 100;
    uint16_t bme_hum = hum_bme * 100;

    uint16_t scd_temp = (temp_scd + 10) * 100;
    uint16_t scd_hum = hum_scd * 100;
    uint16_t scd_co2 = co2_scd;

    uint16_t sds_pm25 = pm25_sds * 10;
    uint16_t sds_pm10 = pm10_sds * 10;

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
    sensor_data[17] = 0x00;
}

void initialize_radio()
// was pin 18
// while(!Serial){}

{
    // reset rn2483
    Serial.println("resetting lora");
    pinMode(PA10, OUTPUT);
    digitalWrite(PA10, LOW);
    Serial.println("done resetting lora");
    // ingestelde appKey en joinEUI
    ;
    // print appKey en joinEUI in serial monitor
    Serial.print("appKey: ");
    Serial.println(appKey);
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
    bool join_result = false;

    join_result = myLora.initOTAA(appEui, appKey);

    // Loopt vast bij OTAA, uncomment voor ABP
    while (!join_result) {
        Serial.println("Unable to join. Are your keys correct, and do you have "
                       "TTN coverage?");
        delay(30000);  // delay a minute before retry
        join_result = myLora.init();
    }
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
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;

    // Set Clock divider for GCLK4
    GCLK->GENDIV.reg =
        GCLK_GENDIV_DIV(
            4) |  // Select clock divisor to divide by 32 (2 ^ (4 + 1))
        GCLK_GENDIV_ID(4);  // GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;  // Wait for synchronization

    // Connect GCLK4 output to RTC
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK4 |  // Select GCLK4
                        GCLK_CLKCTRL_ID_RTC |     // Connect to the RTC
                        GCLK_CLKCTRL_CLKEN;       // Enable GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;  // Wait for synchronization

    // RTC configuration
    // (rtc.h)--------------------------------------------------
    RTC->MODE1.CTRL.bit.ENABLE = 0;  // Disable the RTC
    while (RTC->MODE0.STATUS.bit.SYNCBUSY)
        ;  // Wait for synchronization

    RTC->MODE1.CTRL.bit.SWRST = 1;  // Software reset the RTC
    while (RTC->MODE0.STATUS.bit.SYNCBUSY)
        ;  // Wait for synchronization

    RTC->MODE1.CTRL.reg |=
        RTC_MODE1_CTRL_PRESCALER_DIV1024 |  // Set prescaler to 1024
        RTC_MODE1_CTRL_MODE_COUNT16;        // Set RTC to mode 0, 32-bit timer

    // SET TIME IN SEC HERE AFTER RTC_MODE1_PER_PER
    RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(
        sleepseconds);  // Interrupt time in s: 1Hz/(#seconds + 1)
    while (RTC->MODE0.STATUS.bit.SYNCBUSY)
        ;  // Wait for synchronization

    // Configure RTC interrupts ------------------------------------------
    RTC->MODE1.INTENSET.reg =
        RTC_MODE0_INTENSET_CMP0;  // Enable RTC overflow interrupts

    NVIC_SetPriority(RTC_IRQn, 0);  // Set the Nested Vector Interrupt
                                    // Controller (NVIC) priority for RTC
    NVIC_EnableIRQ(
        RTC_IRQn);  // Connect RTC to Nested Vector Interrupt Controller (NVIC)

    // Enable Deep Sleep
    // Mode--------------------------------------------------------------
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // | SCB_SCR_SLEEPONEXIT_Msk;  // Put
                                        // the SAMD21 in deep sleep upon
                                        // executing the __WFI() function
    NVMCTRL->CTRLB.reg |=
        NVMCTRL_CTRLB_SLEEPPRM_DISABLED;  // Disable auto power reduction during
                                          // sleep - SAMD21 Errata 1.14.2
}

void RTC_Handler() {
    RTC->MODE1.INTFLAG.bit.OVF = 1;  // Reset the overflow interrupt flag
}

void MCUsleep(int downtime) {
    RTC->MODE1.PER.reg =
        RTC_MODE1_PER_PER(downtime);  // Interrupt time in s: 1Hz/(#seconds + 1)
    while (RTC->MODE1.STATUS.bit.SYNCBUSY) {
    };                               // Wait for synchronization
    RTC->MODE1.CTRL.bit.ENABLE = 1;  // Enable the RTC
    while (RTC->MODE1.STATUS.bit.SYNCBUSY) {
    };                                           // Wait for synchronization
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;  // Disable SysTick interrupts
    __DSB();  // Complete outstanding memory operations - not required for
              // SAMD21 ARM Cortex M0+
    __WFI();  // Put the SAMD21 into deep sleep, Zzzzzzzz...
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  // Enable SysTick interrupts

    RTC->MODE1.CTRL.bit.ENABLE =
        0;  // Disable the RTC  --> to stop count outside standby
    while (RTC->MODE1.STATUS.bit.SYNCBUSY) {
    };  // Wait for synchronization
}