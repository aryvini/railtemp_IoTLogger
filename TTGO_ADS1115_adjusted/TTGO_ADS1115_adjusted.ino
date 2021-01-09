

//DataLogger TCC-ARY VINICIUS NERVIS FRIGERI
//2020-05-11 IPB - BRAGANÇA
//This sketch connect to losant using GSM module and TCALL board


//INCLUDE ALL LIBS
#include <Losant.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <Adafruit_ADS1015.h>
#include <OneWire.h>
#include <DallasTemperature.h>



//SENSORS PIN DEFINITIONS
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */


// onewire is used to read the DS18b
#define ONE_WIRE_BUS 14  //GPIO 14
OneWire oneWire(ONE_WIRE_BUS);       //cria um instancia oneWire
DallasTemperature sensors(&oneWire);  //inicialzia uma instancia DallasTemperature com a instancia oneWire
DeviceAddress Tinternal = {0x28, 0xFF, 0x2D, 0x2A, 0x70, 0x16, 0x05, 0xA1}; //variável tipo address para o endereco do DS18b
//OBS: CADA DS18b TEM UM UNICO ENDERECO


//source: https://randomnerdtutorials.com/esp32-sim800l-send-text-messages-sms/

// Losant credentials.
const char *LOSANT_DEVICE_ID = "5eb964849b0687000627fa02";
const char *LOSANT_ACCESS_KEY = "ACCES KEY";
const char *LOSANT_ACCESS_SECRET = "ACCES SECRET";

LosantDevice device(LOSANT_DEVICE_ID);


// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "TM"; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = ""; // SIM card PIN code, if any



// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define TINY_GSM_DEBUG SerialMon
//#define DUMP_AT_COMMANDS

//### THIS INCLUDES MUST BE HERE, OTHERWISE IT WILL NOT COMPILE
//## REASON: UNKOWN
#include <Wire.h>
#include <TinyGsmClient.h>
#include "utilities.h"

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif


TwoWire I2CPower = TwoWire(1);

#define uS_TO_S_FACTOR 1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  280        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00


TinyGsmClient client(modem);
const int  port = 80;

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Keep power when running from battery
  Wire.begin(I2C_SDA, I2C_SCL);
  bool   isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  //########## MODEM INICIALIZATION ######################################################

  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // Or, use modem.init() if you don't need the complete restart

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }

//### MODEM INITIALIZED ############
  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();
  sensors.begin();


// Configure the wake up source as timer wake up
esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

}

void loop() {

  inicialStatus();
  connectLosant();
  
  //Serial.print('Temp: ');
  float Temp_inside,mVavg,Trail;

  Temp_inside = internal_temp(Tinternal);
  mVavg = read_mV_avg_ADS115();
  Trail = Thermocouple_temp(mVavg, Temp_inside);
  
  Serial.print(mVavg,6); Serial.print("      ");Serial.print(Temp_inside);Serial.print("      ");Serial.println(Trail);



  report(mVavg,Temp_inside,Trail); //send state to losant

  
  //Delay before disconnecting, otherwise there is no enough time to losant to receive the data package
  delay(1000);


   // Close client and disconnect
  client.stop();
  SerialMon.println(F("Server disconnected"));
  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));
 
  // Put ESP32 into deep sleep mode (with timer wake up)
  SerialMon.println("Sleeping for 5 minutes");
  esp_deep_sleep_start();



}

float read_mV_avg_ADS115(){
  SerialMon.println("Reading internal thermocouple voltage");
  int ADC;
  float ADC_avg;
  int i;
  float multiplier = 0.0078125;
  float mV;

  ADC=0;
  ADC_avg=0;

  for(i=1;i<=10;i++){
    ADC = ads.readADC_Differential_0_1();
    SerialMon.println(ADC);
    ADC_avg = ADC_avg + ADC;
    delay(200);
  }
  ADC_avg = ADC_avg/10;
  SerialMon.print("ADC=");SerialMon.println(ADC_avg);
  mV = ADC_avg * multiplier;
  SerialMon.print("voltage=");SerialMon.println(mV);

  return mV;
}


float Thermocouple_temp(float mV, float Tinternal){

  float a,b,c,d;
  float Thj;
  a=41.151185814045434;
  b=2406.2643619227174;
  c=2373.6581172797855;
  d=40.61160163624644;

  Thj = (mV + (Tinternal - c)/d)*a+b;
  
 return Thj;
}

float internal_temp(DeviceAddress DSB_address){
  SerialMon.println("Reading internal temp");

  int i;
  float Tint,Tint_avg;
  Tint=0;
  Tint_avg=0;

  for(i=1;i<=3;i++){
    sensors.requestTemperatures();
    Tint = sensors.getTempC(DSB_address);
    while(Tint == -127.00){
      delay(100);
      sensors.requestTemperatures();
      Tint = sensors.getTempC(DSB_address);
    }
    SerialMon.println(Tint);
    Tint_avg=Tint_avg + Tint;
    delay(300); 
  }
  
  Tint_avg=Tint_avg/3;
  SerialMon.print("internal temp=");SerialMon.println(Tint_avg);

  return Tint_avg;
  
}


void inicialStatus(){

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print(F("Connecting to APN: "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    restart();
    return;
  }
  SerialMon.println(" OK");
}

void report(float mV, float inside_temp, float Trail)
{
  // allocate the memory for the document
  const size_t CAPACITY = JSON_OBJECT_SIZE(3);
  StaticJsonDocument<CAPACITY> doc;

  // create an object
  JsonObject object = doc.to<JsonObject>();
  object["mV"] = mV;
  object["inside_temp"] = inside_temp;
  object["Trail"] = Trail;

  // serialize the object and send the result to Serial
  serializeJson(doc, Serial);
  device.sendState(object);
  SerialMon.println("Reported to losant");
}

void connectLosant()
{

  SerialMon.print("Authenticating Device...");

  // Connect to Losant.
  SerialMon.println();
  SerialMon.print("Connecting to Losant...");

  device.connect(client, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);
  delay(100);
  
  int i=0;
    while (!device.connected())
  {
    delay(1000);
    Serial.print(".");
    i++;
    if(i<=30){
    SerialMon.println("Failed to connect to losant");
    SerialMon.println("Failed to connect to losant");
    restart();
    }
  }

  SerialMon.println("Connected!");
  SerialMon.println();
  SerialMon.println("This device is now ready for use!");

}

//Function to restart the board if something goes wrong
void restart(){
  SerialMon.println("Something went wrong");
  SerialMon.println("Restarting board");
  esp_restart();
}
