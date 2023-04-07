#include "SdsDustSensor.h"

SdsDustSensor sds(Serial1);
uint16_t pm25_sds = 0, pm10_sds = 0;
bool sds_status = true;

void setup() {
    Serial.begin(9600);  // serial port to computer
    sds.begin();
    sds.setQueryReportingMode();
}

void loop() {
    sds.wakeup();

    delay(30000);

    PmResult pm = sds.queryPm();
    if (!pm.isOk()) {
        sds_status = false;
    } else {
        pm25_sds = pm.pm25;
        pm10_sds = pm.pm10;
    }

    // turn off sensors
    scd4x.powerDown();

    Serial.println(pm.pm25);
    Serial.println();
    Serial.println(pm.pm10);

    delay(30000);
}