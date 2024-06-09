//TopTilt V2-3, BLE Inclinometer beacon
//idle power consumption at last run (7s avg = 40uA)

//Import libraries
#include <Arduino.h>
#include <bluefruit.h>
#include <SX126x-RAK4630.h>
#include <ZzzMovingAvg.h>
#include <SPI.h>
#include <SCL3300.h>
#include "nrfx.h"
#include "hal/nrf_nvmc.h"
//#include "peripherals/nrf/power.h"

SCL3300 inclinometer;

// Forward declarations for callback functions
void ble_connect_callback(uint16_t conn_handle);
void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason);

//BLE Services setup
BLEUart g_BleUart;
BLEDfu bledfu;
BLEBas blebas;    // BAS (Battery Service) helper class instance
bool g_BleUartConnected = false;

//battery reading parameters
#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.0)      // Compensation factor for the VBAT divider, depend on the board
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
#define battPin PIN_QSPI_IO3

//Globals
char currentMode = 'p';
char lastMode = '0';

unsigned int pauseInterval = 60; //change back to 60, time taken between BLE sends

float accelX = -1.0;
float accelY = -1.0;
float accelXlast = 0.0;
float accelYlast = 0.0;
float accelXzero = 0.0;
float accelYzero = 0.0;
float reportX = -1.0;
float reportY = -1.0;

#define bLED 13
#define wLED 14
#define rLED 15
#define gLED 16

byte battLvl = -1;
int rawBattLvl = -1;

float accelThreshold = 5.0;

int dwellTime = -1;

bool catastrophe = false;
bool significantChange = false;

byte advData[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00};

//Unique ID info for Device Name
typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)

#define DEVICE_ID_HIGH    (*(pREG32 (0x10000060)))
#define DEVICE_ID_LOW     (*(pREG32 (0x10000064)))

//sample averaging setup
#define numSamples 10
ZzzMovingAvg <20, float, float> accelXfiltered;
ZzzMovingAvg <20, float, float> accelYfiltered;

int magReading = 2300;

void setup()
{
  //Functions needed to obtain proper deep sleep levels
  //nrf_peripherals_power_init();
  lora_rak4630_init();
  Radio.Sleep();

  //configure Pins
  pinMode(bLED, OUTPUT); digitalWrite(bLED, LOW);
  pinMode(wLED, OUTPUT); digitalWrite(wLED, HIGH);
  pinMode(rLED, OUTPUT); digitalWrite(rLED, LOW);
  pinMode(gLED, OUTPUT); digitalWrite(gLED, LOW);
  pinMode(WB_A1, INPUT);
  pinMode(battPin, INPUT);


  //Serial.begin(115200);

  //Setup battery voltage sensing
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12); // Can be 8, 10, 12 or 14



  //Configure BLE
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);

  Bluefruit.begin(1, 0);
  Bluefruit.autoConnLed(false);
  //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  // Set the BLE device name
  String IDstring = String(DEVICE_ID_LOW, HEX);
  String nameString = "KTT#" + String(IDstring[0]) + String(IDstring[1]) + String(IDstring[2]) + String(IDstring[3]);
  nameString.toUpperCase();
  Bluefruit.setName(nameString.c_str()); //device name shows like KTT#cb4e, last four characters should be unique

  //Setupcallbacks
  Bluefruit.Periph.setConnectCallback(ble_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(ble_disconnect_callback);

  // Configure and Start BLE Uart Service and DFU service
  bledfu.begin();
  g_BleUart.begin();
  blebas.begin();
  blebas.write(getBattLevel());

  setupAccel();

  //Preemptively fill the averaging buffer
  delay(200);
  //setupAccel();
  getManySamples(10);
  getManySamples(10);


  // Set up and start advertisements
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  //Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addManufacturerData(advData, 8);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(9999, 9999); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode

  updateAdv();

  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds

  //Signal end of setup
  digitalWrite(wLED, LOW);
}



void loop()
{

  BLEloop();
  getManySamples(numSamples);
  sendBLE();
  updateAdv();


}

void BLEloop() {
  if (!g_BleUartConnected) { //Check if we are connected or not
    for (int i = 0; i < pauseInterval; i++) { //not connected, enter sleep loop that periodically checks for a zeroing magnet
      delay(1000);
      checkMag();
    }
    getBattLevel();
  }
  else {
    if (g_BleUart.available() > 0) {
      parseBLEUART();
    }
    else {
      delay(500);
      checkMag();
    }
  }
  digitalWrite(bLED, HIGH);
  delay(25);
  digitalWrite(bLED, LOW);
}

void checkMag() { //Function that checks if a magnet is nearby, and is removed in a timely manner. If not, it's assumed to be interference
    int magNorm = 2300;
    int magDeadzone = 250;
  
    magReading = analogRead(WB_A1); //get reading
  
    if (magReading > magNorm + magDeadzone || magReading < magNorm - magDeadzone) { //check reading against threshold and deadzones
      magReading = analogRead(WB_A1);
      //    g_BleUart.println(magReading); //Debug code
      //    g_BleUart.println("Magnet triggered");
      //    g_BleUart.println(magNorm + magDeadzone);
      //    g_BleUart.println(magNorm - magDeadzone);
      digitalWrite(gLED, HIGH);
      digitalWrite(rLED, LOW);
      digitalWrite(bLED, LOW);
      digitalWrite(wLED, LOW);
      delay(500);
      unsigned long magTime = millis();
      while ((magReading  > magNorm + magDeadzone || magReading < magNorm - magDeadzone) && millis() - magTime < 1500) { //check if magnet is still there
        delay(250);
        g_BleUart.println("delaying...");
        magReading = analogRead(WB_A1);
        g_BleUart.println(magReading);
      }
      if (millis() - magTime >= 1500) { //if "magnet" is still present, likely means it's actually interference, so adjust thresholds accordingly
        g_BleUart.println("rejecting bad magnet, increasing magnet deadzone to rule out future false positives");
        magDeadzone += 100;
        digitalWrite(gLED, LOW);
        digitalWrite(rLED, HIGH);
        delay(100);
        digitalWrite(rLED, LOW);
      }
      else {
        g_BleUart.println("magnet confirmed, zeroing");
        digitalWrite(gLED, LOW);
        delay(50);
        digitalWrite(gLED, HIGH);
        zeroAngles();
        digitalWrite(gLED, LOW);
      }
      digitalWrite(rLED, LOW);
    }
}

int getBattLevel() {
  volatile int reading = 0;

  for (int i = 0; i < 10; i++) {
    reading += analogRead(battPin);
    delay(50);
  }
  reading /= 10;

  reading = constrain(map(reading, 682, 4087, 0, 100), 0, 100); //0% is ~0.5V, technically lower limit for cells is 2.7.

  if (reading <= 10) {
    digitalWrite(rLED, HIGH);
    delay(50);
    digitalWrite(rLED, LOW);
  }

  blebas.write(reading);
  battLvl = reading;
  g_BleUart.println(analogRead(battPin));
  g_BleUart.println(battLvl);
  return reading;
}

void zeroAngles() { //invoked on 'z' command or when a magnet is present
  digitalWrite(gLED, HIGH);
  inclinometer.begin(SPI, WB_SPI_CS);
  accelXzero = 0;
  accelYzero = 0;
  for (int i = 0; i < 50; i++) {
    while (!inclinometer.available()) delay(10);

    accelX = abs(inclinometer.getTiltLevelOffsetAngleX());
    accelY = abs(inclinometer.getTiltLevelOffsetAngleY());
    accelXfiltered.add(accelX);
    accelYfiltered.add(accelY);
    accelXzero = accelXfiltered.get();
    accelYzero = accelYfiltered.get();
  }
  inclinometer.powerDownMode();
  sleepSPI();

  digitalWrite(gLED, LOW);
}

void loopColors() {
  for (int i = 13; i <= 16; i++) {
    g_BleUart.println(i);
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
    delay(1500);
    digitalWrite(i, LOW);
  }
}

void sendBLE() {
  //writes to BLE UART characteristic according to currentMode

  switch (currentMode) {
    case 'r':
      //realtime reporting mode
      while (currentMode == 'r') {
        inclinometer.begin(SPI, WB_SPI_CS); //necessary
        parseBLEUART();
        pollAccel();
        g_BleUart.println(formAccelString());
      }
      break;
    case 'p':
      //periodic reporting mode
      g_BleUart.println(formAccelString());
      break;
    case 'c':
      //on-catastrophe reporting mode
      if (abs(accelX) >= accelThreshold || abs(accelY) >= accelThreshold) {
        g_BleUart.println(formAccelString());
        //g_BleUart.println("Catastrophe!");
      }
      break;
    case 'C':

      //on-signficant-change reporting mode
      //      g_BleUart.print("lastVal: "); g_BleUart.println(accelXlast);
      //      g_BleUart.print("currentVal: "); g_BleUart.println(accelX);
      if (abs(abs(accelX) - abs(accelXlast)) >= accelThreshold || abs(abs(accelY) - abs(accelYlast)) >= accelThreshold) {
        delay(dwellTime);
        getOneSample();
        if (abs(abs(accelX) - abs(accelXlast)) >= accelThreshold || abs(abs(accelY) - abs(accelYlast)) >= accelThreshold) {
          g_BleUart.println(formAccelString());
          //g_BleUart.println("Significant change!");
          accelXlast = accelX;
          accelYlast = accelY;
        }

      }
      break;
    default:
      //unrecognized reporting mode, or mode 'd' which is handled in parseBLEUART
      currentMode = '0';
      break;
  }
}

void parseBLEUART() { //checks if commands are available to be done, and acts accordingly
  if (g_BleUart.available())
  {
    String incoming = g_BleUart.readString();
    lastMode = currentMode;
    currentMode = incoming[0];

    switch (currentMode) {
      case 'z':
        //        getOneSample();
        //        accelXzero = accelX + accelXzero;
        //        accelYzero = accelY + accelYzero;
        zeroAngles();
        currentMode = lastMode;
        break;
      case 'r':
        //realtime reporting mode
        pauseInterval = 0;
        break;
      case 'p':
        //periodic reporting mode
        incoming.remove(0, 1);
        pauseInterval = incoming.toInt();
        break;
      case 'd':
        //on-demand reporting mode
        g_BleUart.println(formAccelString());
        pauseInterval = 5;
        break;
      case 'c':
        //on-catastrophe reporting mode
        incoming.remove(0, 1);
        accelThreshold = incoming.toFloat();
        pauseInterval = 5;
        break;
      case 'm':
        //check magnet
        incoming.remove(0, 1);
        checkMag();
        pauseInterval = 5;
        currentMode = lastMode;
        break;
      case 'C':
        //on-signficant-change reporting mode, thresh[float],dwell[int],pauseInterval[int]
        incoming.remove(0, 1);
        parseChangeCommand(incoming);
        break;
      case 'b': {
          //report battery
          getBattLevel();
          currentMode = lastMode;
        }
        break;
      default:
        //unrecognized reporting mode
        currentMode = '0';
        break;
    }

  }
}

void parseChangeCommand(String incoming) {
  float AaccelThreshold;
  g_BleUart.println("enter threshold (Ex. 5.0)"); //set accelThreshold
  while (!g_BleUart.available()) {
    delay(500);
  }
  String Cincoming = g_BleUart.readString();
  AaccelThreshold = Cincoming.toFloat();

  g_BleUart.println("enter dwell time (milliseconds)"); //set dwellTime in milliseconds
  while (!g_BleUart.available()) {
    delay(500);
  }
  Cincoming = g_BleUart.readString();
  dwellTime = Cincoming.toInt();

  g_BleUart.println("enter sensor polling interval (seconds)"); //set pauseInterval
  while (!g_BleUart.available()) {
    delay(500);
  }
  Cincoming = g_BleUart.readString();
  pauseInterval = Cincoming.toInt();

  g_BleUart.print("threshold: ");  g_BleUart.println(AaccelThreshold);
  g_BleUart.print("dwellTime: ");  g_BleUart.println(dwellTime);
  g_BleUart.print("sensingInterval: ");  g_BleUart.println(pauseInterval);


}

void updateAdv() {
  formAccelString();
  byte xArray[4];
  byte yArray[4];

  *((float *)xArray) = reportX;
  *((float *)yArray) = reportY;

  for (int i = 0; i < 4; i++) {
    advData[i] = xArray[i];
  }
  for (int i = 4; i < 8; i++) {
    advData[i] = yArray[i - 4];
  }
  advData[8] = battLvl;

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  //Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addManufacturerData(advData, 9);
  Bluefruit.Advertising.addName();
}

void getOneSample() {
  int delayTime = 0;
  inclinometer.begin(SPI, WB_SPI_CS);
  delay(delayTime);
  pollAccel();
  inclinometer.powerDownMode();
  sleepSPI();
}

void sleepSPI() {
  SPI.end();
  digitalWrite(WB_SPI_CS, LOW);
  digitalWrite(WB_SPI_MOSI, LOW);
  digitalWrite(WB_SPI_MISO, LOW);
  digitalWrite(WB_SPI_CLK, LOW);
}


void setupAccel() {
  delay(100);
  if (inclinometer.begin(SPI, WB_SPI_CS) == false) {

    digitalWrite(rLED, HIGH);
    delay(250);
    digitalWrite(rLED, LOW);
    delay(250);

  }

}

void pollAccel() {

  while (!inclinometer.available()) delay(10);

  accelX = inclinometer.getTiltLevelOffsetAngleX() - accelXzero;
  accelY = inclinometer.getTiltLevelOffsetAngleY() - accelYzero;
  accelXfiltered.add(accelX);
  accelYfiltered.add(accelY);


}

String formAccelString() {
  String toSend;

  //toSend += "X: ";
  toSend += String(accelX);
  toSend += ", ";
  toSend += String(accelY);
  //  toSend += ": ";
  //  toSend += String(rawBattLvl);

  reportX = accelX;
  reportY = accelY;

  return toSend;
}

void ble_connect_callback(uint16_t conn_handle)
{
  (void)conn_handle;
  g_BleUartConnected = true;

  //Serial.println("BLE client connected");
  //delay(1000);
  g_BleUart.println("TopTilt PoC ONLINE!");
  g_BleUart.println("Waiting for commands...");
  g_BleUart.println("TopTilt PoC ONLINE!");
  g_BleUart.println("Waiting for commands...");
  g_BleUart.println("TopTilt PoC ONLINE!");
  g_BleUart.println("Waiting for commands...");

}

void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;
  g_BleUartConnected = false;

  //Serial.println("BLE client disconnected");
}

void getManySamples(int samples) {
  //inclinometer.begin(A5); //V1.5 uses A5, V1.0 uses _CS
  inclinometer.begin(SPI, WB_SPI_CS);
  for (int i = 0; i < samples; i++) {
    pollAccel();
  }
  inclinometer.powerDownMode();
  sleepSPI();
}

void nrf_peripherals_power_init(void) {
  // Set GPIO reference voltage to 3.3V if it isn't already. REGOUT0 will get reset to 0xfffffff
  // if flash is erased, which sets the default to 1.8V
  // This matters only when "high voltage mode" is enabled, which is true on the PCA10059,
  // and might be true on other boards.
  if (NRF_UICR->REGOUT0 == 0xffffffff && NRF_POWER->MAINREGSTATUS & 1) {
    // Expand what nrf_nvmc_word_write() did.
    // It's missing from nrfx V2.0.0, and nrfx_nvmc_word_write() does bounds
    // checking which prevents writes to UICR.
    // Reported: https://devzone.nordicsemi.com/f/nordic-q-a/57243/nrfx_nvmc-h-api-cannot-write-to-uicr
    NRF_NVMC->CONFIG = NRF_NVMC_MODE_WRITE;
    while (!(NRF_NVMC->READY & NVMC_READY_READY_Msk)) {
    }
    NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos;
    __DMB();
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
    }
    NRF_NVMC->CONFIG = NRF_NVMC_MODE_READONLY;

    // Must reset to enable change.
    NVIC_SystemReset();
  }
}
