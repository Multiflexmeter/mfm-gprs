/*
  Multiflexmeter Bangladesh stable Sketch v1.4.1 (still under development)
  Software and hardware design by @Mick van Eerd

  This is the sketch used by the Multiflexmeters in Bangladesh. These devices measure
  waterlevel and temperature. Multiflexmeter will transmit this data to a server through
  mobile internet connectivity.

  Based on previous software design by @Tim van Osch (Pollex') and uses a highly adapted Adafruit FONA library
  Multiflexmeter is open source, it is open to be reproduced adapted but it must stay open source.
  For more info visit www.multiflexmeter.nl or wiki.multiflexmeter.net
*/
#include <LowPower.h>
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <ArduinoJson.h>
#include <RunningMedian.h>
#include <avr/power.h>
#include <rBase64.h>
#include <DallasTemperature.h>

#define MEGA_SERIAL

// Uncomment to print debug info to the serial
#define DEBUGGING
#define DEBUG
//#define CON_INFO

//#ifdef DEBUG
//#define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
//#define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
//#else
//#define DPRINT(...)     //now defines a blank line
//#define DPRINTLN(...)   //now defines a blank line
//#endif

///=============
/// Uncomment lines to disable certain instruments
///=============
//#define USE_BASIC_AUTH

///=============
/// Pin associations
///=============
#define SIM7600_RX_PIN 10
#define SIM7600_TX_PIN 11
#define DEBUG_RX_PIN 8
#define DEBUG_TX_PIN 9
#define JSN_RX_PIN 21 //ard TX
#define JSN_TX_PIN 24 //ard RX
//#define MEASURE_ENABLE_PIN 13 //for start ext. power measurement
#define PWRKEY 31
#define FET_SIM 3
#define FET_JSN 1
#define LED 0
#define FTDI_DTR 22
#define FTDI_CTS 29
#define I2C_SCL 16
#define I2C_SDA 17
#define DS18B20_PIN 15
#define BATT_PIN 25   //25 or A1

///=============
/// Instrment settings
///=============
#define JSN_MEDIAN 15
#define RECEIVE_TIMEOUT_JSN 90 //timeout for reading JSN distance sensor
#define JSN_CALIBRATE 2        //calibrate sensor in cm
#define PWR_ON_TIME 500
#define PWR_OFF_TIME 2600   //time to pull simmodule to off state
#define SEND_TIME 0         //boolean for sending send-time to server
#define JSN_WAKEUP_TIME 300 //actually 200ms but add 100 for stability and mosfet turnon.
#define VOLT_DIV_FACTOR 2360.6557   //divide measured battery voltage with (adcvalue / 1023 * 5000mV)


///=============
/// General settings
///=============
const uint32_t MEASURE_INTERVAL_TIME_S = (60 * 30);                         // Sleep time in seconds
const uint16_t DISTANCE_SEND_MARGIN = 2;                                    // In cm, the difference in distance before sending a new value
const uint32_t SEND_AT_LEAST_EVERY_X_SECONDS = (60UL * 60UL * 12UL);        // Send at least every x seconds even if margin is not reached
#define MAX_NETWORK_CONNECTION_RETRIES 45                                   // Amount of connection retries
// Uncomment next line if the SIM does not require a pincode
//#define SIM_LOCKED
//#define DO_TEMP_COMPENSATION                                              //compensate for the speed of sound with the measured temperature



// Do not change unless you know what you're doing
// This is the device it's ID. This is the id which will be stored in the database
const char MFM_ID[] = "<mfm-id>";
const char PINCODE[] = "0000";
const char server_address[] = "https://portal.multiflexmeter.net/api/v1/networks/gprs/up/";
const char server_token[] = "Bearer <Portal Key>";  //keep "Bearer" (inc. space)
const char APN[] = "internet";
// Change to providers APN settings
//vodafone:       "live.vodafone.com"
//KPN:            "internet"
//T-Mobile:       "smartsites.t-mobile"
//Grameenphone:   "gpinternet"





// Serial communication to the GSM module
//SoftwareSerial simSerial(GSM_RX_PIN, GSM_TX_PIN);
#ifdef SOFTWARE_SERIAL
//SoftwareSerial fonaS =
SoftwareSerial * fonaSerial = new SoftwareSerial(SIM7600_RX_PIN, SIM7600_TX_PIN);
#endif
#ifdef HARDWARE_SERIAL
HardwareSerial fonaS(2);
HardwareSerial *fonaSerial = &fonaS;
#endif
#ifdef MEGA_SERIAL
HardwareSerial &fonaS = Serial1;
HardwareSerial *fonaSerial = &fonaS;
#define Debug Serial
#endif
#ifdef NEWSOFT_SERIAL
SoftwareSerial fonaS(2, 3);
SoftwareSerial *fonaSerial = &fonaS;
#endif
#ifdef HW_SERIAL0
HardwareSerial &fonaS = Serial;
HardwareSerial *fonaSerial = &fonaS;
SoftwareSerial Debug(2, 3);
#endif

uint32_t count = 0;

//Serialinterface to JSN distance sensor, must be in mode 3 configured on board
SoftwareSerial JSNV2(JSN_TX_PIN, JSN_RX_PIN);

//DS18B20
OneWire oneWire(DS18B20_PIN);
DallasTemperature DS18B20(&oneWire);

//#define BUF_LEN 255
//// this is a large buffer for replies
//char replybuffer[255];
Adafruit_FONA fona = Adafruit_FONA(-1);

//sequence of struct should be same sequence with set profile on server.
struct Packet
{
  unsigned short distance;
  float temperature;
  uint16_t battery_mV;
};

///================
/// Below are all functions
///================

//SoftwareSerial is not shutdown by PowerDown sleep mode,
//so write LOW and HIGH the IO pins te prevent power draw in PowerDown
//Powerdraw is 20mA to 29 uA.
void PowerJSN(bool onoff)
{
  if (onoff)
  {
    digitalWrite(JSN_TX_PIN, HIGH);
    digitalWrite(JSN_RX_PIN, HIGH);
    delay(20);
    digitalWrite(FET_JSN, HIGH);
    delay(JSN_WAKEUP_TIME);
    JSNV2.begin(9600);
    delay(20);
  }
  else
  {
    JSNV2.end();
    digitalWrite(FET_JSN, LOW);
    delay(20);
    digitalWrite(JSN_RX_PIN, LOW);
    digitalWrite(JSN_TX_PIN, LOW);
  }
}
void PowerSIM(bool onoff)
{
  if (onoff)
  {
    //   digitalWrite (SIM7600_RX_PIN, HIGH);
    //   digitalWrite (SIM7600_TX_PIN, HIGH);
    //delay(20);
    digitalWrite(FET_SIM, HIGH);
    delay(100);
    fonaSerial->begin(115200);
  }
  else
  {
    fonaSerial->end();
    digitalWrite(FET_SIM, LOW);
    delay(20);
    // digitalWrite (SIM7600_RX_PIN, LOW);
    //  digitalWrite (SIM7600_TX_PIN, LOW);
  }
}
///
/// Enters a deep sleep for <MEASURE_INTERVAL_TIME_S> amount of time
///
void EnterDeepSleep()
{
  PrintSleepTimeInfo();
#ifdef DEBUGGING
  Debug.print(String("Sleeping ") + (int)((float)MEASURE_INTERVAL_TIME_S / 8) + " cycles of 8s...");
  Debug.print("count:");
  Debug.println(++count);

  Debug.end();
  delay(20);
#endif
  // Sleep is at most 8S, so loop DESIRED_TIME/8 times
  for (int i = 0; i < (int)((float)MEASURE_INTERVAL_TIME_S / 8); i++)
  {
    // Enter low power mode
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  //delay(1000);
#ifdef DEBUGGING
  delay(200);
  Debug.begin(9600);
  Debug.println("Wake up!");
#endif
}

//call with delay
//read serial mode JSN ultasonic distance sensor
uint16_t ReadJSN()
{
  byte buforek[4];
  unsigned long start;

  JSNV2.write(0x55); //request distance measurement
  delay(5);
  start = millis();
  JSNV2.flush();
  while (JSNV2.available() <= 0)
  { //wait for data
    delay(10);
    if ((start + RECEIVE_TIMEOUT_JSN) < millis())
    { //but dont wait too long
      DEBUG_PRINT("TimeOUT JSN");
      return 0;
    }
  }
  for (int i = 0; i < 4; i++)
  { //read 4 bytes which contain distance data
    buforek[i] = (byte)JSNV2.read();
    delay(2);
  }

  if (buforek[0] == 0xff && ((buforek[0] + buforek[1] + buforek[2]) & 0x00ff) == buforek[3]) //SUM check
  {
    //Two's complement make 16 bit int and convert mm to cm
    return (((uint16_t)buforek[1] << 8) + buforek[2] + JSN_CALIBRATE) / 10;
  }
  DEBUG_PRINTLN("NO valid data Available.");
  return 0;
}

//measuring waterdistance and passing it to json
//return false not handled in callfunction
uint16_t WaterDistance()
{
  RunningMedian distances = RunningMedian(JSN_MEDIAN);

  PowerJSN(true);
  for (int i = 0; i < JSN_MEDIAN; i++)
  {
    uint16_t dist = ReadJSN();
    Debug.print(i);
    Debug.print(":Distance: ");
    Debug.println(dist);
    distances.add(dist);
  }
  PowerJSN(false);

  uint16_t dist_med = distances.getMedian();
  Debug.print("Distance median: ");
  Debug.println(dist_med);
  return dist_med;
}

uint16_t CompensateJSNDistance(uint16_t distance, float temp) {
  return (uint16_t)((331.30f + 0.606f * temp) / 340.00f * distance);
}

//Read battery voltage, return value in mVolts.
uint16_t ReadBatteryVoltage() {
  uint16_t raw_value = analogRead(BATT_PIN);
  return (uint16_t)((raw_value / 1023.00) * 5.00 * VOLT_DIV_FACTOR);
}

//Read temperature from DS18B20
float ReadTemperature()
{
  DS18B20.requestTemperatures();
  //already being done in requestTemperatures(); blockTillConversionComplete(12); //12bit default bit resolution, read temperature duration/conversion = 750ms.
  return DS18B20.getTempCByIndex(0);
}

void flushSerial()
{
  while (Debug.available() > 0)
  {
    char t = Debug.read();
  }
}

void flushSim()
{
  while (fonaSerial->available() > 0)
  {
    char t = fonaSerial->read();
  }
}

bool retrieveStatus(FONAFlashStringPtr cmd)
{
  const uint8_t ans_size = 64;
  char ans[ans_size];

  if (!fona.getReplyPrt(cmd, ans, 2, ans_size))
  {
    // Debug.println("could not extract netstatus from response");
    return false;
  }
  //Debug.print(cmd); Debug.print("::"); Debug.println(ans);
  long response = strtol(ans, NULL, 0);
  if (response == 1 || response == 5)
  {
    //Debug.print(cmd); Debug.print("::"); Debug.println("connected");
    return true;
  }
  return false;
}

bool CheckNetwork()
{
  int tries = 0;
  while (!retrieveStatus(F("AT+CREG?")))
  {
    delay(500);
    tries++;
    if (tries > MAX_NETWORK_CONNECTION_RETRIES)
    {
      // Debug.println("cannot connect, giving up.");
      return false;
    }
  }

  tries = 0;
  while (!retrieveStatus(F("AT+CGREG?")))
  {
    delay(500);
    tries++;
    if (tries > MAX_NETWORK_CONNECTION_RETRIES)
    {
      //Debug.println("cannot connect, giving up.");
      return false;
    }
  }
  return true;
}

bool SetupNetwork()
{
  if (!CheckNetwork())
    return false;
  if (!fona.OpenInternet(true))
    return false;
  return true;
}

bool StartSIM()
{
  PowerSIM(true);
  if (!fona.begin(*fonaSerial))
  {
    Debug.println("could not start sim");
    PowerSIM(false);
    return false; // Don't use if statement because an OK reply could be sent incorrectly at 115200 baud
  }
  flushSim();
  Debug.flush();
  //Debug.print("trying to set APN:"); Debug.println(APN);
  fona.setNetworkSettings(APN);

  //unlock sim
#ifdef SIM_LOCKED
  if (!fona.unlockSIM(PINCODE)) {
    Debug.println("Could not unlock SIM");
    PowerSIM(false);
    return false;
  }
  Debug.println("\tSIM UNLOCKED");
#endif
  return true;
}

///main send function
int last_distance = INT16_MIN;
uint32_t send_atleast_countdown = SEND_AT_LEAST_EVERY_X_SECONDS;
bool SendDataToServer()
{
  bool delivered = false;

  // Do meassurements
  float temperature = ReadTemperature();
  uint16_t median_distance = WaterDistance();
  uint16_t comp_distance = CompensateJSNDistance(median_distance, temperature);
  uint16_t battery_mV = ReadBatteryVoltage();

  uint16_t to_send_distance;
#ifdef DO_TEMP_COMPENSATION
  to_send_distance = comp_distance;
  Debug.print("Sending distance compensated: ");
#else
  to_send_distance = median_distance;
  Debug.print("Sending distance raw: ");
#endif
  Debug.println(to_send_distance);

  Debug.print("Distance median: "); Debug.print(median_distance);
  Debug.print("\t Distance compensated: "); Debug.print(comp_distance);
  Debug.print("\t Temperature: "); Debug.print(temperature);
  Debug.print("\t Battery mV: "); Debug.println(battery_mV);

  Packet packet {
    to_send_distance,
    temperature,
    battery_mV
  };

  //Evaluate if measurements need to be sent to server
  send_atleast_countdown -= MEASURE_INTERVAL_TIME_S;
  // If distance does not exceed margin then don't continue and we can still wait
  if (send_atleast_countdown > 0 && abs(to_send_distance - last_distance) < DISTANCE_SEND_MARGIN)
  {
    return false;
  }

  digitalWrite(LED, HIGH);
  if (!StartSIM())
    return false;

  //if (!fona.CheckModuleON()) return false;
  DynamicJsonBuffer jsonBuffer(1024);
  JsonObject &json_data = jsonBuffer.createObject();
  if (SetupNetwork())
  {
    json_data["mfm_id"] = MFM_ID; //might be dangerous doing it this way
    json_data["type"] = "measurements";


    //char payload[64];
    //memcpy(payload, (void *)&packet, sizeof(packet));
    if (rbase64.encode((byte *)&packet, sizeof(Packet)) != RBASE64_STATUS_OK) return false;
    json_data["payload"] = rbase64.result();
    int json_len = json_data.measureLength();

#ifdef DEBUG
    Debug.print("size of json: ");
    Debug.println(json_len);
    json_data.printTo(Debug);
    Debug.println();
#endif

    //print json to char
    char measurement_data[json_len + 1];
    json_data.printTo(measurement_data, sizeof(measurement_data));

#ifdef DEBUG
    //Debug.print("measurement_data: "); Debug.println(measurement_data);
#endif

    //token length
    uint16_t token_len = sizeof(server_token) / sizeof(server_token[0]);
    Debug.print("token length: "); Debug.println(token_len);

    //Do HTTP POST to server
    delivered = fona.HTTPPOST(measurement_data, json_len, server_address, server_token, token_len);
    if (delivered)
    {
      last_distance = to_send_distance;
      // Reset atleast every x seconds timer
      send_atleast_countdown = SEND_AT_LEAST_EVERY_X_SECONDS;
      //  Debug.println("Succesfully send message to server");
    }
  }
#ifdef DEBUG
  else
  {
    Debug.println("Could not bring up connection");
  }
#endif

  //show some connection values
#ifdef CON_INFO
  uint8_t buf_size = 255;
  char con_info[buf_size];
  CheckConInfo(con_info, buf_size);
  CheckServiceTechn();
  uint8_t rssi = fona.getRSSI(); //gets printed by debug
#endif

  PowerSIM(false);
  delay(5);
  //  if (!time_to_send) // digitalWrite(MEASURE_ENABLE_PIN, HIGH);
  digitalWrite(LED, LOW);
  return delivered;
}

void PrintSleepTimeInfo() {
  Debug.print("Max interval time: "); Debug.println(SEND_AT_LEAST_EVERY_X_SECONDS);
  Debug.print("Min interval time: "); Debug.println(MEASURE_INTERVAL_TIME_S);
  Debug.print("Time left before next forced transmission: "); Debug.println(send_atleast_countdown);
  Debug.print("Number of measurements left: "); Debug.print(send_atleast_countdown / MEASURE_INTERVAL_TIME_S); Debug.print('/'); Debug.println(SEND_AT_LEAST_EVERY_X_SECONDS / MEASURE_INTERVAL_TIME_S);
}


int8_t CheckServiceTechn()
{
  uint8_t ans_size = 64;
  char ans[ans_size];

  if (!fona.getReplyPrt(F("AT+CNSMOD?"), ans, 2, ans_size))
  {
    // Debug.println("could not retrieve netmode, something went wrong..");
    return -1;
  }
  fona.readline(5000); //eat OK

  // Debug.print("netmode: "); Debug.println(ans);
  long netmode = strtol(ans, NULL, 0);
  /*
    0 – no service
    1 – GSM                           2G
    2 – GPRS                          2.5G
    3 – EGPRS (EDGE)                  2.75G
    4 – WCDMA                         3G
    5 – HSDPA only(WCDMA)             3G or 3.5G
    6 – HSUPA only(WCDMA)             3G or 3.75G
    7 – HSPA (HSDPA and HSUPA, WCDMA) 3G / 3.5G / 3.75G
    8 – LTE                           4G
    9 – TDS - CDMA                    3G
    10 – TDS - HSDPA only             3.5G
    11 – TDS - HSUPA only             3.75G
    12 – TDS - HSPA (HSDPA and HSUPA) 3.5G / 3.75G
    13 – CDMA                         2G
    14 – EVDO                         3G
    15 – HYBRID (CDMA and EVDO)       2G / 3G
    16 – 1XLTE(CDMA and LTE)          2G / 4G
    23 – eHRPD                        4G / 2G LTE over CDMA
    24 – HYBRID(CDMA and eHRPD)       4G / 2G
  */

  Debug.print("Netmode: ");
  switch (netmode)
  {
    case 0:
      Debug.println("NO Service");
      break;
    case 1:
      Debug.println("2G: GSM");
      break;
    case 2:
      Debug.println("2.5G: GPRS");
      break;
    case 3:
      Debug.println("2.75G: EGPRS (EDGE)");
      break;
    case 4:
      Debug.println("3G: WCDMA");
      break;
    case 5:
      Debug.println("3G or 3.75G: HSDPA only(WCDMA)");
      break;
    case 6:
      Debug.println("3G or 3.75G: HSUPA only(WCDMA)");
      break;
    case 7:
      Debug.println("3G/3.5G/3.75G: HSPA (HSDPA and HSUPA, WCDMA)");
      break;
    case 8:
      Debug.println("4G: LTE");
      break;
    case 9:
      Debug.println("3G: TDS-CDMA");
      break;
    case 10:
      Debug.println("3.5G: TDS - HSDPA only");
      break;
    case 11:
      Debug.println("3.75G: TDS - HSUPA only");
      break;
    case 12:
      Debug.println("3.5G/3.75G: TDS - HSPA (HSDPA and HSUPA)");
      break;
    case 13:
      Debug.println("2G: CDMA");
      break;
    case 14:
      Debug.println("3G: EVDO");
      break;
    case 15:
      Debug.println("2G/3G: HYBRID (CDMA and EVDO)");
      break;
    case 16:
      Debug.println("2G/4G: 1XLTE(CDMA and LTE)");
      break;
    case 23:
      Debug.println("4G/2G (LTE over CDMA): eHRPD");
      break;
    case 24:
      Debug.println("2G/4G: HYBRID(CDMA and eHRPD)");
      break;
    default:
      Debug.println("Did not recognise netmode");
      break;
  }
  return netmode;
}

//AT+CPSI?
//+CPSI: LTE,Online,204-16,0x0036,14346250,367,EUTRAN-BAND3,1750,5,5,-100,-927,-620,19
bool CheckConInfo(char *c, uint8_t s)
{
  uint8_t readbytes = fona.CheckSysInfo(F("AT+CPSI?"), c, s);

  if (readbytes <= 0)
  {
    //  Debug.println("Could not request sys info");
    return false;
  }
#ifdef DEBUG
  Debug.print("c: ");
  Debug.println(c);
#endif

  return true;
}

///
/// Initial setup for when we power up.
///
void setup()
{
  //give ds18b20 time to startup and do its first measurement
  DS18B20.begin();

#ifdef DEBUGGING
  Debug.begin(9600);
  //Serial1.begin(9600);
  Debug.print("MFM-GPRS System start up...");
#endif

  //disable unused modules and IO, do not use, not stable. Draws actually more current when enabled.
  //SetPowerSaveFunctions();
  //SetPowerSaveIO();

  // Setup pin pinmodes
  pinMode(FET_JSN, OUTPUT);
  pinMode(FET_SIM, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BATT_PIN, INPUT);

  PowerJSN(false);
  PowerSIM(false);

#ifdef DEBUGGING
  Debug.println("Finished");
#endif


}

///
/// The never ending loop-da-loop
///
void loop()
{
  SendDataToServer();
  delay(500);
  //go to sleep
  EnterDeepSleep();
}

void SetPowerSaveFunctions()
{
  SPCR = 0;   //disable SPI
  ADCSRA = 0; // disable ADC

  power_adc_disable();
  power_spi_disable();
  //power_timer1_disable();
  //power_timer2_disable();
  power_timer3_disable();
  power_twi_disable();
}

//set as digital output and write LOW
void SetPowerSaveIO()
{
  int i = 0;
  for (i = 0; i < 32; i++)
  {
    //#define SIM7600_RX_PIN 10
    //#define SIM7600_TX_PIN 11
    //#define DEBUG_RX_PIN 8
    //#define DEBUG_TX_PIN 9
    //#define JSN_RX_PIN 21 //ard TX
    //#define JSN_TX_PIN 24 //ard RX
    ////#define MEASURE_ENABLE_PIN 13 //for start ext. power measurement
    //#define PWRKEY 31
    //#define FET_SIM 3
    //#define FET_JSN 2
    //#define LED 0
    //#define FTDI_DTR 22
    //#define FTDI_CTS 29
    ////#define I2C_SCL 16
    ////#define I2C_SDA 17
    //#define DS18B20_DQ 15

    if (i == SIM7600_RX_PIN ||
        i == SIM7600_TX_PIN ||
        i == DEBUG_RX_PIN ||
        i == DEBUG_TX_PIN ||
        i == JSN_RX_PIN ||
        i == JSN_TX_PIN ||
        i == PWRKEY ||
        i == FET_SIM ||
        i == FET_JSN ||
        i == LED ||
        i == FTDI_DTR ||
        i == FTDI_CTS ||
        i == DS18B20_PIN)
    {
      continue;
    }
    else
    {
      //if not used io, write low
      Debug.println(i);
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }
}
