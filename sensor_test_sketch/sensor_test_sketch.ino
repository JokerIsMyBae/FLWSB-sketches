#include "SdsDustSensor.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include <rn2xx3.h>

const char* appEui = "0000000000000000";
const char* appKey = "5E7773DF01C66243843429D3B38C5FCB";

/*
  Vraag temp, pressure en humidity op van BME
  en format als 5 bytes in een array meegegeven als parameter.
  | Byte nr | Name            | Sensor range     | On Node MCU | Reformat |
  | ------- | --------------- | ---------------- | ----------- | -------- |
  | 0-1     | Temperature     | -40 tot 85°C     | +40 *100    | /100 -40 | -
  BME280 | 2-4     | Pressure        | 300 tot 1100 hPa | *100        | /100 | -
  BME280 | 5-6     | Humidity        | 0 tot 100%       | *100        | /100 | -
  BME280 | 7-8     | Temperature     | -10 tot 60°C     | +10 *100    | /100 -10
  | - SCD41 | 9-10    | co2             | 400 tot 5000 ppm | *100        | /100
  | - SCD41 | 11-12   | Humidity        | 0 tot 95 %       | *100        | /100
  | - SCD41 | 13-14   | PM2.5           | 0 tot 999 μg/m   | *10         | /10
  | - SDS011 | 15-16   | PM10            | 0 tot 999 μg/m   | *10         | /10
  | - SDS011

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
Adafruit_BME280 bme;
SdsDustSensor sds(Serial1);

uint16_t co2_scd = 0, pm25_sds = 0, pm10_sds = 0;
float temp_bme = 0.0f, pres_bme = 0.0f, hum_bme = 0.0f, temp_scd = 0.0f,
      hum_scd = 0.0f;
byte sensor_data[14];  // 13 bytes + one byte for error check
unsigned status;

// LoRa variables
rn2xx3 myLora(Serial2);
double last_send_time = 0;
double interval = 60000;  // one minute

void setup() {
    Serial.begin(57600);   // serial port to computer
    Serial2.begin(57600);  // serial port to radio

    Wire.begin();
    scd4x.begin(Wire, 0x62);
    status = bme.begin();
    if (status)
        bme.setSampling(Adafruit_BME280::sensor_mode::MODE_FORCED);
    sds.setQueryReportingMode();

    Serial.println("Startup");

    initialize_radio();

    // transmit a startup message
    myLora.tx("TTN Mapper on TTN Enschede node");
}

void loop() {

    if (millis() - last_send_time >= interval) {
        last_send_time = millis();

        // measurements
        sensor_data[13] = 0x00;  // clear all errors
        executeMeasurements();

        formatData();

        // printing data (optional)
        for (int i = 0; i < 13; i++) {
            Serial.println(sensor_data[i], HEX);
        }

        // send over LoRa
        Serial.println("TXing");
        double start = millis();

        myLora.txBytes(sensor_data,
                       14);  // give data and data length; check declaration
        double transmission = millis() - start;
        Serial.println(transmission);
    }
}

bool checkError(error) {
    if (error) {
        temp_scd = 0xFFFF;
        co2_scd = 0xFFFF;
        hum_scd = 0xFFFF;
        return true;
    }
    return false;
}

void executeMeasurements() {
    bool isDataReady = false;
    uint16_t serialnr0, serialnr1, serialnr2, error;
    uint8_t i = 0;

    // Wake-up sequence
    error = scd4x.wakeUp();
    if (checkError(error))
        return;
    // SCD41 needs 20ms to wake up, delay is in library currently - this will be
    // changed.
    // delay(20);

    sds.wakeup();
    // SDS011 needs to measure for 3 sec until measurements are reliable.
    // delay(30000);

    error = scd4x.getSerialNumber(serialnr0, serialnr1, serialnr2);
    if (checkError(error))
        return;

    // Send out start measurement commands
    error = scd4x.measureSingleShot();
    if (checkError(error))
        return;
    // measureSingleShot has built-in delay of 5000ms to wait
    // for measurements - this will be changed.
    do {
        error = scd4x.getDataReadyFlag(isDataReady);
        if (checkError(error))
            return;
        i++;
    } while ((!isDataReady) && (i < 5));
    error = scd4x.readMeasurement(co2_scd, temp_scd, hum_scd);
    if (checkError(error))
        return;

    if (!status) {
        temp_bme = 0xFFFF;
        pres_bme = 0xFFFFFF;
        hum_bme = 0xFFFF;
        sensor_data[13] |= (1 << 7);  // error setting up BME
    } else {
        isDataReady = bme.takeForcedMeasurement();
        // takeforcedmeasurement has built-in timeout check - this will be
        // changed.
        if (!isDataReady) {
            temp_bme = 0xFFFF;
            pres_bme = 0xFFFFFF;
            hum_bme = 0xFFFF;
        }
        // After timeout check read measurements
        temp_bme = bme.readTemperature();
        pres_bme = bme.readPressure() / 100.0F;
        hum_bme = bme.readHumidity();
        if (isnan(temp_bme) || isnan(pres_bme) || isnan(hum_bme)) {
            temp_bme = 0xFFFF;
            pres_bme = 0xFFFFFF;
            hum_bme = 0xFFFF;
            sensor_data[13] |= (111 << 4);
        }
    }

    // After 30 sec query the measurement results of SDS011
    PmResult pm = sds.queryPm();
    if (!pm.isOk()) {
        pm25_sds = 0xFFFF;
        pm10_sds = 0xFFFF;
        return;
    }
    pm25_sds = pm.pm25;
    pm10_sds = pm.pm10;

    // Sleep sensors, bme sleeps automatically
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
