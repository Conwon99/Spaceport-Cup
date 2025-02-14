/* ESP32 LoRa Flight Computer
 * Author : Connor Dorward
 * 
 * Logs barometer, GPS and accelerometer data. Sends data via LoRa.
 * Activates main and drogue parachutes on barometer data.
 * Bluetooth configurable.
 * 
 * 
 */

TaskHandle_t LoRa_Tx;
TaskHandle_t read_sensors;


#include <LoRa.h>
#define BAND 866E6

float seconds =0;
char secs_c[10];

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

/// BLUETOOTH//////////////////////////////////////////////////////////////////
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

//GPS//////////////////////////////////////////////////////////////////////
#include <SoftwareSerial.h>
#include <TinyGPS.h> 

TinyGPS gps; // create gps object 
SoftwareSerial serialGPS(25,12);

float lat,lon; // create variable for latitude and longitude object  

char lat_c[20];
char lon_c[20];



// SD CARD
#include "FS.h"
#include "SD.h"
#include "SPI.h"


#define SD_CS 13
#define SD_CLK 14
#define SD_MOSI 15
#define SD_MISO 2

bool SDfail=0;

char csv_buffer[100];

SPIClass sdSPI(HSPI);



// BAROMETER 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C


//ACCELEROMETER////////////////////////////////
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Function prototypes//////////////////////////////

float get_alt();
float get_accel();
void get_Readings();
void set_Offset();


void setupSD(void);
void writeFile(fs::FS &fs, const char,const char);
void appendFile(fs::FS &fs, const char , const char);
void log_SD(void);

String BT_receive(int num_msgs);
void BT_send_status();




// GLOBAL VARIABLES ///////////////////////////////
float avg_altitude=0;
float avg_accel;
char accel_c[10];
float altitude_sum;
float alt_offset;
char alt_c[10];

float accel_sum,accel_offset;
int readings = 0;


void setup() 
{

  //Start USB comms
   Serial.begin(9600);
   delay(100);

   //Start GPS comms
   serialGPS.begin(9600);

   // Start Bluetooth comms   
   SerialBT.begin("ESP32"); //Bluetooth device name
   Serial.println("The device started, now you can pair it with bluetooth!");

   //Start LoRa comms
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND))
  {
    Serial.println("Starting LoRa failed!");
  }


// Start Barometer comms
  unsigned status;
  status = bmp.begin();
  if (!status) 
  {
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

   

    // Start Accelerometer comms
    if (!mpu.begin())
    {
    Serial.println("Failed to find MPU6050 chip");
    }

     // set accelerometer range to +-16G
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Start SD card comms
  setupSD();

  set_Offset();

  Serial.println("Effset has been set");
  get_Readings();


   

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    LoRa_Tx_code,   /* Task function. */
                    "LoRa_Tx",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &LoRa_Tx,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 


    xTaskCreatePinnedToCore(
                    read_sensors_code,   /* Task function. */
                    "read_sensor",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &read_sensors,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

 

  //while(BT_receive(1) != "start"){}

 
    
}



//Reads accelerometer data at 10Hz
void read_sensors_code( void * pvParameters )
{
  Serial.print("Task accel_read running on core ");
  Serial.println(xPortGetCoreID());

  for(;;)
  {

    
    for (int i = 0; i <10;i++)  // Takes 10 readings at a rate of every 0.01s (a t100Hz)
  {
  
      altitude_sum = altitude_sum + get_alt();              // BAROMETER DATA
      accel_sum = accel_sum + get_accel();                  // ACCELEROMETER DATA
      vTaskDelay(10 / portTICK_PERIOD_MS);

  }

  //average the 10 readings over the 0.1s
  avg_altitude = altitude_sum/10;
  avg_accel = accel_sum/10;
  
  //reset the sum values for next reading
  altitude_sum =0;
  accel_sum = 0;
  
//
//  Serial.print("Alt=");
//  Serial.println(avg_altitude);
//
//  Serial.print("Lat=");
//  Serial.println(lat);
//
//  Serial.print("Lon=");
//  Serial.println(lon);

//  Serial.println();

  

    // Convert sensor reading into strings to send to SD/Lora
    sprintf(secs_c, "%g", seconds);
    sprintf(lat_c, "%g", lat);
    sprintf(lon_c,"%g", lon);
    sprintf(alt_c,"%g", avg_altitude);
    sprintf(accel_c,"%g", avg_accel);
    
    //Append strings to buffer
   // 'Time, Lat, Lon, Altitude, Accel, state'
    strcpy(csv_buffer, "");
    strcat(csv_buffer, secs_c);
    strcat(csv_buffer, ",");
    strcat(csv_buffer, lat_c);
    strcat(csv_buffer, ",");
    strcat(csv_buffer, lon_c);
    strcat(csv_buffer, ",");
    strcat(csv_buffer,alt_c);
    strcat(csv_buffer, ",");
    strcat(csv_buffer,accel_c);
  
    //Ensure buffer correct
    Serial.print("csv_buffer:");
    Serial.println(csv_buffer);

    //Send buffer to SD
    appendFile(SD, "/data.txt", csv_buffer); // Cant include a logSD function as GPS outputs lat=0 lon=0
    appendFile(SD, "/data.txt", "\n"); // Cant include a logSD function as GPS outputs lat=0 lon=0
    seconds=seconds+0.1;

  }

}



//Sends Lora Data every Second
void LoRa_Tx_code( void * pvParameters )
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;)
  {

     Serial.print("Sending Packet:");
     Serial.println(seconds);
     LoRa.beginPacket();
     LoRa.print("PacketNum:");
     LoRa.println(seconds);
     LoRa.println(csv_buffer);
     LoRa.endPacket();

     vTaskDelay(1000 / portTICK_PERIOD_MS); // Send data every second


  } 
}


void loop() 
{

 // GPS works best if kept in main loop
 while (serialGPS.available() > 0) 
      {
       // Serial.print("h");
  
        if(gps.encode(serialGPS.read()))// encode gps data 
        {  
        gps.f_get_position(&lat,&lon); // get latitude and longitude 
      
        // display position 
        Serial.print("Position: "); 
        Serial.print("Latitude:"); 
        Serial.print(lat,6); 
        Serial.print(";"); 
        Serial.print("Longitude:"); 
        Serial.println(lon,6);  
        }
        
      }

}




///////////FUNCTIONS////////////////////////////////////////////////////



float get_alt()
{
   return bmp.readAltitude(1013.25)-alt_offset;
}


float get_accel()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  return a.acceleration.x - accel_offset;
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

  //get_coords();

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

  Serial.print("Alt=");
  Serial.println(avg_altitude);

  Serial.print("Lat=");
  Serial.println(lat);

  Serial.print("Lon=");
  Serial.println(lon);

  Serial.println();

}



void set_Offset()
{

// Calculate Offset 
for (int i = 0; i <100;i++)  // Take 10 readings then average
  {
  
      //accel_x_sum = accel_x_sum + accel_x; // BAROMETER DATA
      accel_sum = accel_sum + get_accel();                  // ACCELEROMETER DATA

     delay(10);

  }
    accel_offset = accel_sum/100;
    accel_sum=0;

    Serial.print("accel_offset = ");
    Serial.println(accel_offset);
  
}





void log_SD()
{
// "Time , height, lat, lon , accel, state"


appendFile(SD, "/data.txt", "yowassuop");

}

void setupSD() {
  
  Serial.println("Initializing SD card...");
sdSPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
if (!SD.begin(SD_CS, sdSPI)) {

Serial.println("Card Mount Failed");

SDfail=1;

}

else

{
Serial.println("Success");
SDfail = 0;
}
uint8_t cardType = SD.cardType();
if(cardType == CARD_NONE) {
  Serial.println("No SD card attached");

  SDfail=1;

}

File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "Vbat, Iin, Iout \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    SDfail = 1;
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
    SDfail = 1;
  }
  file.close();
}


void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    SDfail = 1;
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
    SDfail = 1;
  }
  file.close();
}
