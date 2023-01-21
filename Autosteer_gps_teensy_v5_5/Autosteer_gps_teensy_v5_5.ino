// Single antenna, IMU, & dual antenna code for AgOpenGPS
// If dual right antenna is for position (must enter this location in AgOpen), left Antenna is for heading & roll
//
// connection plan:
// Teensy Serial 2 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 2 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
// Teensy Serial 7 RX (7) to F9P Heading receiver TX1 (Relative position from left antenna to right antenna)
// Teensy Serial 7 TX (8) to F9P Heading receiver RX1 (not used)
// F9P Position receiver TX2 to F9P Heading receiver RX2 (RTCM data for Moving Base)
//
// Configuration of receiver
// Position F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 115200
// Serial 1 In - RTCM (Correction Data from AGO)
// Serial 1 Out - NMEA GGA/VTG
// CFG-UART2-BAUDRATE 115200 is good, keep all the same as UART1
// Serial 2 Out - RTCM 1074,1084,1094,1230,4072.0 (Correction data for Heading F9P, Moving Base)  //1124 is not needed (China’s BeiDou system) - Save F9P brain power 
//
// Heading F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 115200
// Serial 1 Out - UBX-NAV-RELPOSNED
// CFG-UART2-BAUDRATE 115200 is good, keep all the same
// Serial 2 In RTCM

/************************* User Settings *************************/
// Serial Ports
#define SerialAOG Serial
#define SerialGPSA &Serial2
#define SerialGPSB &Serial7

HardwareSerial *SerialGPS = SerialGPSA;
HardwareSerial *SerialGPS2 = SerialGPSB;


const int32_t baudAOG = 115200;
const int32_t baudGPS = 115200;

//RTCM serial radio input
//add here



// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
  #include "zNativeEthernet.h"
#endif // ARDUINO_TEENSY41

#define ImuWire Wire        //SCL=19:A5 SDA=18:A4

// Swap BNO08x roll & pitch?
const bool swapRollPitch = false;
//const bool swapRollPitch = true;

// is the GGA the second sentence?
// send GPS data via  0 = USB, 1 = Ethernet 
int send_Data_Via = 0;

int GGAReceivedLED = 13;

/*****************************************************************/

#include "zNMEAParser.h"
#include <Wire.h>
#include "zBNO08x_AOG.h"






byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
unsigned long prev_PWM_Millis = 0;
byte velocityPWM_Pin = 36;      // Velocity (MPH speed) PWM pin
int velocityPWM_Multiplier = 10; // PWM (MPH * multiplier)

bool useDual = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

// booleans to see if we are using CMPS or BNO08x
bool useCMPS = false;
bool useBNO08x = false;

//CMPS always x60
#define CMPS14_ADDRESS 0x60

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

//Dual
double headingcorr = 900;  //90deg heading correction (90deg*10)

float baseline;
double baseline2;
float rollDual;
float rollDualRaw;
double relPosD;
double relPosDH;
double heading = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];   //Extra serial tx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<1> parser;

//BNO08x, time after last GPS to load up IMU ready data for the next Panda takeoff
const uint16_t IMU_DELAY_TIME = 90; //Best results seem to be 90-95ms
uint32_t lastTime = IMU_DELAY_TIME;
uint32_t gpsCurrentTime = IMU_DELAY_TIME;

//BNO08x, how offen should we get data from IMU (The above will just grab this data without reading IMU)
const uint16_t GYRO_LOOP_TIME = 10;
uint32_t lastGyroTime = GYRO_LOOP_TIME, lastPrint;

//CMPS14, how long should we wait with GPS before reading data from IMU then takeoff with Panda
const uint16_t CMPS_DELAY_TIME = 4;  //Best results seem to be around 5ms
uint32_t gpsReadyTime = CMPS_DELAY_TIME;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; //Auto set off in autosteer setup
bool swapSerialPorts = false;  //Swap serial GPS ports if backwards
bool GGA_Available = false;     //Do we have GGA on correct port?
uint32_t PortSwapTime = 0;

//100hz summing of gyro
float gyro, gyroSum;
float lastHeading;

float roll;
float pitch;
float bno08xHeading = 0;
int16_t bno08xHeading10x = 0;

// Setup procedure ------------------------
void setup()
{
  pinMode(GGAReceivedLED, OUTPUT);

  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);

  delay(10);
  Serial.begin(baudAOG);
  delay(10);
  Serial.println("***************************************");
  Serial.println("**************Start setup**************");
  Serial.println("***************************************");

  Serial.println("\r\n  Initializing Serial...");
  
  SerialGPS->begin(baudGPS);
  SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  delay(10);
  SerialGPS2->begin(baudGPS);
  SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
  SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);

  
  EthernetSetup();
  
  autosteerSetup();

  //IMUSetup();
  Serial.println("\r\n  Initializing IMU...");
  //test if CMPS working
  uint8_t error;

  ImuWire.begin();
  
      //Serial.println("Checking for CMPS14");
  ImuWire.beginTransmission(CMPS14_ADDRESS);
  error = ImuWire.endTransmission();

  if (error == 0)
  {
      //Serial.println("Error = 0");
      Serial.print("    CMPS14 ADDRESs: 0x");
      Serial.println(CMPS14_ADDRESS, HEX);
      Serial.println("    CMPS14 Ok.");
      useCMPS = true;
  }
  else
  {
    //Serial.println("Error = 4");
    //Serial.println("    CMPS not Connected or Found");
  }

  if (!useCMPS)
  {
    for (int16_t i = 0; i < nrBNO08xAdresses; i++)
    {
      bno08xAddress = bno08xAddresses[i];

      //Serial.print("\r\nChecking for BNO08X on ");
      //Serial.println(bno08xAddress, HEX);
      ImuWire.beginTransmission(bno08xAddress);
      error = ImuWire.endTransmission();

      if (error == 0)
      {
        //Serial.println("Error = 0");
        Serial.print("    0x");
        Serial.print(bno08xAddress, HEX);
        Serial.println(" BNO08X Ok.");
        
        // Initialize BNO080 lib
        if (bno08x.begin(bno08xAddress, ImuWire))
        {
          ImuWire.setClock(400000); //Increase I2C data rate to 400kHz

          // Use gameRotationVector
          bno08x.enableGyro(GYRO_LOOP_TIME);
          bno08x.enableGameRotationVector(GYRO_LOOP_TIME - 1); //Send data update every REPORT_INTERVAL in ms for BNO085, looks like this cannot be identical to the other reports for it to work...

          // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
          if (bno08x.getFeatureResponseAvailable() == true)
          {
            if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, (GYRO_LOOP_TIME - 1)) == false); //bno08x.printGetFeatureResponse();

            // Break out of loop
            useBNO08x = true;
            break;
          }
          else
          {
                 Serial.println("    BNO08x init fails!!");
          }
        }
        else
        {
                 Serial.println("    BNO080 not detected at given I2C address.");
        }
      }
      else
      {
                 //Serial.println("Error = 4");
                 //Serial.print("    0x");
                 //Serial.print(bno08xAddress, HEX);
                 //Serial.println(" BNO08X not Connected or Found");
      }
    }
  }

  delay(10);
  if (useCMPS)
  {
    Serial.print("    useCMPS = true");
  }
  else if (useBNO08x)
  {
    Serial.print("    useBNO08x = true");
  }
  else {
    Serial.println("    No IMU Connected or Found");
  }



  Serial.println("\r\n***************************************");
  Serial.println("***************End setup***************");
  Serial.println("***************************************");
  Serial.println("\r\nwaiting for GPS...\r\n");
}

void loop()
{
  #ifdef ARDUINO_TEENSY41
    Ethernet.packetLOOP();
  #endif // ARDUINO_TEENSY41

  if(GGA_Available == false) 
  {
    if (systick_millis_count - PortSwapTime >= 10000)
    {
      Serial.println("Swapping GPS ports...\r\n");
      swapSerialPorts = !swapSerialPorts;
      PortSwapTime = systick_millis_count;
      SerialGPS = swapSerialPorts ? SerialGPSA : SerialGPSB;
      SerialGPS2 = swapSerialPorts ? SerialGPSB : SerialGPSA;
    }
  }

  //Read incoming nmea from GPS
  if (SerialGPS->available())
  {
    parser << SerialGPS->read();
  }
  
  //Pass NTRIP etc to GPS
  if (SerialAOG.available())
  {
    SerialGPS->write(SerialAOG.read());
  }
  
  //If both dual messages are ready, send to AgOpen
  if (dualReadyGGA == true && dualReadyRelPos == true)
  {
    // TODO (jaapvandenhandel) should be build PAOGI?
    BuildNmea();
    dualReadyGGA = false;
    dualReadyRelPos = false;
  }
  //If anything comes in SerialGPS2 RelPos data
  if (SerialGPS2->available())
  {
    uint8_t incoming_char = SerialGPS2->read();  //Read RELPOSNED from F9P

    // Just increase the byte counter for the first 3 bytes
    if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount])
    {
      relposnedByteCount++;
    }
    else if (relposnedByteCount > 3)
    {
      // Real data, put the received bytes in the buffer
      ackPacket[relposnedByteCount] = incoming_char;
      relposnedByteCount++;
     
      // Check the message when the buffer is full
      if (relposnedByteCount > 71)
      {
        if (calcChecksum())
        {
          //if(deBug) Serial.println("RelPos Message Recived");
          useDual = true;
          relPosDecode();
        }
//        else if(deBug) Serial.println("ACK Checksum Failure: ");

        relposnedByteCount = 0;
      }
    }
    else
    {
      // Reset the counter, becaues the start sequence was broken
      relposnedByteCount = 0;
    }
  }

  //IMU timming section
  if (useBNO08x)
  {
    gpsCurrentTime = systick_millis_count;
    if (isTriggered && gpsCurrentTime - lastTime >= IMU_DELAY_TIME)
    {
      // Load up BNO08x data from gyro loop ready for takeoff
      imuHandler();

      // reset the timer
      isTriggered = false;
    }

    gpsCurrentTime = systick_millis_count;
    if (gpsCurrentTime - lastGyroTime >= GYRO_LOOP_TIME)
    {
      // Get data from BNO08x (Gyro, Heading, Roll, Pitch)
      // Get data from CMPS14 (Gyro Only)
      GyroHandler(gpsCurrentTime - lastGyroTime);
    }
  }
  else if (useCMPS)
  {
    gpsCurrentTime = systick_millis_count;
    if (isTriggered && gpsCurrentTime - gpsReadyTime >= CMPS_DELAY_TIME)
    {
      imuHandler(); //Get data from CMPS and gyro loop ready for takeoff
      BuildNmea(); //Send Panda

      //reset the timer
      isTriggered = false;
    }
  }

  if(Autosteer_running) autosteerLoop();
  
}//End Loop

//**************************************************************************

bool calcChecksum()
{
  CK_A = 0;
  CK_B = 0;

  for (int i = 2; i < 70; i++)
  {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}
