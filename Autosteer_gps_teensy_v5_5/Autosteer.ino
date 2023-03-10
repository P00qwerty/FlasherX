/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

////////////////// User Settings /////////////////////////

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 5.0

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 5100

// Address of CMPS14 shifted right one bit for arduino wire library
#define CMPS14_ADDRESS 0x60

// BNO08x definitions
#define REPORT_INTERVAL 90 //Report interval in ms (same as the delay at the bottom)

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE  4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM  2

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM  3

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37

//Define sensor pin for current or pressure sensor
#define ANALOG_SENSOR_PIN A0

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

#include "zBNO08x_AOG.h"

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//show life in AgIO
uint8_t helloAgIO[] = {0x80, 0x81, 0x7f, 0xC7, 1, 0, 0x47 };
uint8_t helloCounter = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t AOG[] = {0x80, 0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };
int16_t AOGSize = sizeof(AOG);

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
  uint8_t Kp = 40;  //proportional gain
  uint8_t lowPWM = 10;  //band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;//max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;     //sent as percent
};  Storage steerSettings;  //11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0; //if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;  //1 if switch selected
  uint8_t SteerButton = 0;  //1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
};  Setup steerConfig;          //9 bytes

//reset function
void(* resetFunc) (void) = 0;

void autosteerSetup()
{
  Serial.println("\r\n  Initializing AutoSteer...");
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0)
  {
    analogWriteFrequency(PWM1_LPWM, 490);
    analogWriteFrequency(PWM2_RPWM, 490);
  }
  else if (PWM_Frequency == 1)
  {
    analogWriteFrequency(PWM1_LPWM, 122);
    analogWriteFrequency(PWM2_RPWM, 122);
  }
  else if (PWM_Frequency == 2)
  {
    analogWriteFrequency(PWM1_LPWM, 3921);
    analogWriteFrequency(PWM2_RPWM, 3921);
  }

  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  if (steerConfig.CytronDriver) pinMode(PWM2_RPWM, OUTPUT);

  //set up communication
  Wire1.begin();
    
  // Check ADC 
  if(adc.testConnection())
  {
    Serial.println("    ADC Connecton OK");
  }
  else
  {
    Serial.println("    ADC Connecton FAILED!");
    Autosteer_running = false;
  }

  //50Khz I2C
  //TWBR = 144;   //Is this needed?

  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
  }
  else
  {
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, steerConfig);
  }

  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;


  if(Autosteer_running && ARDUINO_TEENSY41)
  {
    Serial.println("    Autosteer running, waiting for AgOpenGPS via UDP/Ethernet");
  }
  else
  {
    Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
    if(!ARDUINO_TEENSY41)Serial.println("    Ethernet not available");
    Serial.println("    Autosteer disabled, GPS only mode");
    return;
  }

  adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

}// End of Setup

void autosteerLoop()
{

  //Serial.println("AutoSteer loop");

  // Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = systick_millis_count;

  if (currentTime - autsteerLastTime >= LOOP_TIME)
  {
    autsteerLastTime = currentTime;

    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch

    if (steerConfig.SteerSwitch == 1)         //steer switch on - off
    {
      steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
    }
    else if (steerConfig.SteerButton == 1)    //steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH)
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
    }
    else                                      // No steer switch and no steer button
    {
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
      }

      // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
      if (guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
      {
        steerSwitch = 1;
        previous = 0;
      }
    }

    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
      int16_t analogValue = analogRead(ANALOG_SENSOR_PIN);

      // When the current sensor is reading current high enough, shut off
      if (abs(((analogValue - 512)) / 10.24) >= steerConfig.PulseCountMax) //amp current limit switch off
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
      int16_t analogValue = analogRead(ANALOG_SENSOR_PIN);

      // Calculations below do some assumptions, but we should be close?
      // 0-250bar sensor 4-20ma with 150ohm 1V - 5V -> 62,5 bar/V
      // 5v  / 1024 values -> 0,0048828125 V/bit
      // 62,5 * 0,0048828125 = 0,30517578125 bar/count
      // 1v = 0 bar = 204,8 counts
      int16_t steeringWheelPressureReading = (analogValue - 204) * 0.30517578125;

      // When the pressure sensor is reading pressure high enough, shut off
      if (steeringWheelPressureReading >= steerConfig.PulseCountMax)
      {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
      }
    }

    remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    /*
      #if Relay_Type == 1
        SetRelays();       //turn on off section relays
      #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
      #endif
    */

    //get steering position
    if (steerConfig.SingleInputWAS)   //Single Input ADS
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
      steeringPosition = adc.getConversion();
      adc.triggerConversion();//ADS1115 Single Mode

      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
    }
    else    //ADS1115 Differential Mode
    {
      adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
      steeringPosition = adc.getConversion();
      adc.triggerConversion();

      steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
    }

    //DETERMINE ACTUAL STEERING POSITION

    //convert position to steer angle. 32 counts per degree of steer pot position in my case
    //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS)
    {
      steeringPosition = (steeringPosition - 6805  - steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    }
    else
    {
      steeringPosition = (steeringPosition - 6805  + steerSettings.wasOffset);   // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      //Enable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 0);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 1);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 1);

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      calcSteeringPID();  //do the pid
      motorDrive();       //out to motors the pwm value
    }
    else
    {
      //we've lost the comm to AgOpenGPS, or just stop request
      //Disable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 1);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 0);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2

      pwmDrive = 0; //turn off steering motor
      motorDrive(); //out to motors the pwm value
      pulseCount = 0;
    }

    /* TODO: Still needed?
      //send empty pgn to AgIO to show activity
      if (++helloCounter > 10)
      {
      SendUDPPacket(helloAgIO);
      helloCounter = 0;
      } */
  } //end of timed loop

  //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  //delay(1);

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp(IPAddress src_ip, uint16_t src_port, uint8_t* udpData, uint16_t len)
{
  //  Serial.print("ReceiveUdp: ");
  //  Serial.println(len);

  if (len > 13)
  {
    if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
    {
      if (udpData[3] == 0xFE)  //254
      {
        gpsSpeed = ((float)(udpData[5] | udpData[6] << 8)) * 0.1;

        guidanceStatus = udpData[7];

        //Bit 8,9    set point steer angle * 100 is sent
        steerAngleSetPoint = ((float)(udpData[8] | ((int8_t)udpData[9]) << 8)) * 0.01; //high low bytes

        //Serial.print("steerAngleSetPoint: ");
        //Serial.println(steerAngleSetPoint);

        //Serial.println(gpsSpeed);

        if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
        {
          watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
        }
        else          //valid conditions to turn on autosteer
        {
          watchdogTimer = 0;  //reset watchdog
        }

        //Bit 10 Tram
        tram = udpData[10];

        //Bit 11
        relay = udpData[11];

        //Bit 12
        relayHi = udpData[12];

        //----------------------------------------------------------------------------
        //Serial Send to agopenGPS

        int16_t sa = (int16_t)(steerAngleActual * 100);

        AOG[5] = (uint8_t)sa;
        AOG[6] = sa >> 8;

        // heading
        AOG[7] = (uint8_t)9999;
        AOG[8] = 9999 >> 8;

        // roll
        AOG[9] = (uint8_t)8888;
        AOG[10] = 8888 >> 8;

        AOG[11] = switchByte;
        AOG[12] = (uint8_t)pwmDisplay;

        //checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < AOGSize - 1; i++)
          CK_A = (CK_A + AOG[i]);

        AOG[AOGSize - 1] = CK_A;

        //off to AOG
        SendUDPPacket(AOG, AOGSize);

        // Stop sending the helloAgIO message
        helloCounter = 0;

        //Serial.println(steerAngleActual);
        //--------------------------------------------------------------------------
      }

      //steer settings
      else if (udpData[3] == 0xFC)  //252
      {
        //PID values
        steerSettings.Kp = ((float)udpData[5]);   // read Kp from AgOpenGPS

        steerSettings.highPWM = udpData[6]; // read high pwm

        steerSettings.lowPWM = (float)udpData[7];   // read lowPWM from AgOpenGPS

        steerSettings.minPWM = udpData[8]; //read the minimum amount of PWM for instant on

        steerSettings.steerSensorCounts = udpData[9]; //sent as setting displayed in AOG

        steerSettings.wasOffset = (udpData[10]);  //read was zero offset Lo

        steerSettings.wasOffset |= (udpData[11] << 8);  //read was zero offset Hi

        steerSettings.AckermanFix = (float)udpData[12] * 0.01;

        //crc
        //udpData[13];

        //store in EEPROM
        EEPROM.put(10, steerSettings);

        // for PWM High to Low interpolator
        highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
      }

      else if (udpData[3] == 0xFB)  //251 FB - SteerConfig
      {
        uint8_t sett = udpData[5]; //setting0

        if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
        if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
        if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
        if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
        if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
        if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
        if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
        if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

        steerConfig.PulseCountMax = udpData[6];

        //was speed
        //udpData[7];

        sett = udpData[8]; //setting1 - Danfoss valve etc

        if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
        if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
        if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;

        //crc
        //udpData[13];

        EEPROM.put(40, steerConfig);

        //reset the arduino
        resetFunc();

      }//end FB

    } //end if 80 81 7F
  }
}
#endif


//ISR Steering Wheel Encoder
void EncoderFunc()
{
  if (encEnable)
  {
    pulseCount++;
    encEnable = false;
  }
}
