

/* ESP32 LoRa Flight Computer
 * Author : Connor Dorward
 * 
 * Logs barometer, GPS and accelerometer data. Sends data via LoRa.
 * Activates main and drogue parachutes on barometer data.
 * Bluetooth configurable.
 * 
 * 
 */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

bool BT_data_collected = false;
char BT_message[5][30] = {"","","",""};     
int BT_message_num=0;
int BT_num_messages = 1;
int charnum = 0;
char incomingChar;




#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

// Logging Data
float avg_altitude=0;
float avg_accel;
float altitude_sum, accel_sum;
float alt_offset;


float get_alt()
{
   Serial.print(F("Approx altitude = "));
   Serial.print(bmp.readAltitude(1013.25)-alt_offset); /* Adjusted to local forecast! */
   Serial.println(" m");

   return bmp.readAltitude(1013.25)-alt_offset;
   
}
float get_accel()
{
  
}

String BT_receive(int num_msgs) {

while(!BT_data_collected)   // Run until all data
{
  if (SerialBT.available()) 
  {

    incomingChar = SerialBT.read();
    int result = iscntrl(incomingChar);

     if (result==0) // If  not a 'hidden' control character, append to message
     {
     BT_message[BT_message_num][charnum] = incomingChar;
     }

     charnum++;

     if (incomingChar == '\n')
     {

       Serial.println(BT_message[BT_message_num]);
       BT_message_num++;
       charnum =0;

      if (BT_message[BT_message_num] =="done" || BT_message_num >=num_msgs ) // prevent buff overflow
      {
        BT_data_collected =true;
      }
      
     }
   
  }
  delay(20);

}

return BT_message[0];
  
}


void BT_send_status() 
{

  //SerialBT.write(avg_alt);
  SerialBT.write(int(avg_altitude));
  SerialBT.write('m');
  //SerialBT.write("\n");
  
}



void get_Readings()

{

  for (int i = 0; i <10;i++)  // Take 10 readings then average
  {
  
      altitude_sum = altitude_sum + get_alt(); // BAROMETER DATA
      accel_sum = accel_sum + get_accel();                  // ACCELEROMETER DATA

     delay(100);

  }

  avg_altitude = altitude_sum/10;
  avg_accel = accel_sum/10;

  altitude_sum =0;
  accel_sum=0;

}



void setup() 
{
  Serial.begin(115200);
  SerialBT.begin("ESP32"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  delay(5000);

  unsigned status;
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  while(BT_receive(1) != "start"){}
  

    
  
  //ADD a 'begin' BT message

  get_Readings();
  alt_offset= avg_altitude;
  BT_send_status();  // Sends status data
  Serial.print(avg_altitude);

//SerialBT.write(avg_altitude)
}

void loop() {

get_Readings();

}
