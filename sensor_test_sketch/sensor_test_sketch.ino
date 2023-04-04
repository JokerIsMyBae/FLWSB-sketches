#include <SensirionI2CScd4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <rn2xx3.h>

//LoRa variables
const char *appEui = "0000000000000000";
const char *appKey = "0BCEC12E4AEFE30F4E336184C1263975";

rn2xx3 myLora(Serial2);
double last_send_time = 0;
double interval = 60000; // one minute

/*
  Vraag temp, pressure en humidity op van BME 
  en format als 5 bytes in een array meegegeven als parameter.

  | Byte nr | Name            | Sensor range     | On Node MCU | Reformat |
  | ------- | --------------- | ---------------- | ----------- | -------- |
  | 0-1     | Temperature     | -40 tot 85°C     | +40 *100    | /100 -40 | - BME280
  | 2-4     | Pressure        | 300 tot 1100 hPa | *100        | /100     | - BME280
  | 5-6     | Humidity        | 0 tot 100%       | *100        | /100     | - BME280
  | 7-8     | Temperature     | -10 tot 60°C     | +10 *100    | /100 -10 | - SCD41
  | 9-10    | co2             | 400 tot 5000 ppm | *100        | /100     | - SCD41
  | 11-12   | Humidity        | 0 tot 95 %       | *100        | /100     | - SCD41
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
  status = bme.begin();
  
  if (status)
    bme.setSampling(Adafruit_BME280::sensor_mode::MODE_FORCED);
}

void loop() {
  //LoRa
  Serial.begin(57600); //serial port to computer
  Serial2.begin(57600); //serial port to radio
  Serial.println("Startup");

  initialize_radio();

  //transmit a startup message
  myLora.tx("TTN Mapper on TTN Enschede node");

  delay(2000);

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

  if (millis() - last_send_time >= interval)
  {
    last_send_time = millis();
    Serial.println("TXing");
    double start = millis();
    //data = "ABCD_ABCD_ABCD_ABCD_ABCD_ABCD_ABCD_ABCD"
    //byte data[5] = {0x0, 0x17,  0x4, 0x4C, 0x50};

    myLora.txBytes(sensor_data, 13); 
    //calculate duration of transmission function
    //double transmission = millis() - start;
    //Serial.println(transmission);
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
  bme.takeForcedMeasurement();
  temp_bme = bme.readTemperature();
  pres_bme = bme.readPressure() / 100.0F;
  hum_bme = bme.readHumidity();
  if (isnan(temp_bme) || isnan(pres_bme) || isnan(hum_bme)) {
    temp_bme = 0xFFFF;
    pres_bme = 0xFFFFFF;
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

void initialize_radio()
{
  //reset rn2483
  Serial.println("resetting lora");
  pinMode(PA10, OUTPUT);
  digitalWrite(PA10, LOW);
  delay(1000);
  digitalWrite(PA10, HIGH);
  Serial.println("done resetting lora")
  ;
  // print appKey en joinEUI in serial monitor
  Serial.print("appKey: ");
  Serial.println(appKey);
  Serial.print("joinEUI: ");
  Serial.println(appEui);
  
  delay(100); //wait for the RN2xx3's startup message
  Serial2.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(myLora.hweui());
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;


  join_result = myLora.initOTAA(appEui, appKey);

  // Loopt vast bij OTAA, uncomment voor ABP
  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(30000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
  
}
