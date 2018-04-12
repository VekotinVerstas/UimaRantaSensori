#include "settings.h"
#include <Sodaq_RN2483.h>
#include <Wire.h>
#include <Sodaq_SHT2x.h>
#include <BH1750.h>
#include "BlueDot_BME280.h"
#include <ArduinoJson.h>

#define debugSerial SerialUSB
#define loraSerial Serial2

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

//Use OTAA, set to false to use ABP
bool OTAA = false;

char TempLabel[3]="T:";
char ConductivityLabel[3]="C:";
char PayloadToSend[20];

BH1750 lightMeter;
BlueDot_BME280 bme280 = BlueDot_BME280();


void setup()
{
  Wire.begin();
  lightMeter.begin();
  // Check bme280 example for details
  bme280.parameter.communication = 0;                    //Choose communication protocol
  bme280.parameter.sensorMode = 0b11;                    //Choose sensor mode
  bme280.parameter.I2CAddress = 0x76;                    //Choose I2C Address  
  // For some reason these are needed
  bme280.parameter.humidOversampling = 0b101;            //Setup Humidity Oversampling
  bme280.parameter.tempOversampling = 0b101;             //Setup Temperature Ovesampling
  bme280.parameter.pressOversampling = 0b101;            //Setup Pressure Oversampling 
  
  delay(1000);
  while ((!debugSerial) && (millis() < 10000)){
  // Wait 10 seconds for debugSerial to open
  }

  debugSerial.println("Start");

  // Start streams
  debugSerial.begin(115200);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

  LoRaBee.setDiag(debugSerial); // to use debug remove //DEBUG inside library
  LoRaBee.init(loraSerial, LORA_RESET);

  //Use the Hardware EUI
  getHWEUI();

  if (bme280.init() != 0x60)
  {    
      debugSerial.println("FAIL");  

  } else {
      debugSerial.println("BME ok");  
  }


  // Print the Hardware EUI
  debugSerial.print("LoRa HWEUI: ");
  for (uint8_t i = 0; i < sizeof(DevEUI); i++) {
    debugSerial.print((char)NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(DevEUI[i])));
    debugSerial.print((char)NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(DevEUI[i])));
  }
  debugSerial.println();  
  
  setupLoRa();
}

void setupLoRa(){
  if(!OTAA){
    // ABP
    setupLoRaABP();
  } else {
    //OTAA
    setupLoRaOTAA();
  }
  // Uncomment this line to for the RN2903 with the Actility Network
  // For OTAA update the DEFAULT_FSB in the library
  // LoRaBee.setFsbChannels(1);

  LoRaBee.setSpreadingFactor(9);
}

void setupLoRaABP(){  
  if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true))
  {
    debugSerial.println("Communication to LoRaBEE successful.");
  }
  else
  {
    debugSerial.println("Communication to LoRaBEE failed!");
  }
}

void setupLoRaOTAA(){
  
  if (LoRaBee.initOTA(loraSerial, DevEUI, AppEUI, AppKey, true))
  {
    debugSerial.println("Network connection successful.");
  }
  else
  {
    debugSerial.println("Network connection failed!");
  }
}

void loop()
{
     delay(2000);
   debugSerial.print(F("Temperature in Celsius:\t\t")); 
   debugSerial.println(bme280.readTempC());
 
   debugSerial.print(F("Temperature in Fahrenheit:\t")); 
   debugSerial.println(bme280.readTempF());
   
   debugSerial.print(F("Humidity in %:\t\t\t")); 
   debugSerial.println(bme280.readHumidity());

   debugSerial.print(F("Pressure in hPa:\t\t")); 
   debugSerial.println(bme280.readPressure());
   delay(600000);

   //StaticJsonBuffer<148> jsonBuffer;
   DynamicJsonBuffer  jsonBuffer(200);
   JsonObject& json = jsonBuffer.createObject();
   String StrToSend;
      
   String reading = getTemperature();
   debugSerial.println(reading);
   int analog_value = analogRead(A0);
   debugSerial.println(analog_value);

   json["temp_in"] = reading;

   // If SHT21 is connected we'll send those readings too
   float temp_out = SHT2x.GetTemperature();
   if (temp_out > -273) {
     json["temp_out"] = temp_out;
   }
   float humi = SHT2x.GetHumidity();
   if (humi > 0) {
     json["humi"] = humi;
   }

   uint16_t lux = lightMeter.readLightLevel();
   if (lux < 54612) {
      json["lux"] = lux;
   }
   // debugSerial.println(lux);
   
   //json["conduct"] = analog_value;
/*
   size_t printTo(char* buffer, size_t size) const;
   size_t printTo(char buffer[size]) const;
   size_t printTo(Print &) const;
   size_t printTo(String &) const;
   size_t printTo(std::string &) const;
*/
   //StrToSend = String(TempLabel)+reading+","+String(ConductivityLabel)+String(analog_value,DEC);
   json.printTo(StrToSend);
   debugSerial.print("Send: ");
   debugSerial.println(StrToSend);
   // Send the payload
   switch (LoRaBee.send(1, (uint8_t*)StrToSend.c_str(), StrToSend.length()))
    {
    case NoError:
      debugSerial.println("Successful transmission.");
      break;
    case NoResponse:
      debugSerial.println("There was no response from the device.");
      break;
    case Timeout:
      debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
      delay(20000);
      break;
    case PayloadSizeError:
      debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! The program will reset the RN module.");
      setupLoRa();
      break;
    case Busy:
      debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
      delay(10000);
      break;
    case NetworkFatalError:
      debugSerial.println("There is a non-recoverable error with the network connection. The program will reset the RN module.");
      setupLoRa();
      break;
    case NotConnected:
      debugSerial.println("The device is not connected to the network. The program will reset the RN module.");
      setupLoRa();
      break;
    case NoAcknowledgment:
      debugSerial.println("There was no acknowledgment sent back!");
      break;
    default:
      break;
    }
    // Delay between readings
    // 60 000 = 1 minute
    delay(10000); 
}

String getTemperature()
{
  //10mV per C, 0C is 500mV
  float mVolts = (float)analogRead(TEMP_SENSOR) * 3300.0 / 1023.0;
  float temp = (mVolts - 500.0) / 10.0;
  
  return String(temp);
}

/**
* Gets and stores the LoRa module's HWEUI/
*/
static void getHWEUI()
{
  uint8_t len = LoRaBee.getHWEUI(DevEUI, sizeof(DevEUI));
}

