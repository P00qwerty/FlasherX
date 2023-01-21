// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41

  #define FlasherXOTA

  #include "zNativeEthernet.h"
  
  uint8_t mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xB3, 0x1B}; // original
  IPAddress ip(192, 168, 5, 111);
  IPAddress ipDestination = { 192, 168, 5, 255 };
  uint16_t AOGNtripPort = 8888;      // local port to listen on
  uint16_t AOGAutoSteerPort = 7777;      // local port to listen on
  uint16_t FlashPort = 6666;      // local port to listen on
  uint16_t portDestination = 9999; //AOG port that listens
#endif // ARDUINO_TEENSY41



void EthernetSetup()
{
  #ifdef ARDUINO_TEENSY41
    Serial.println("\r\n  Initializing Ethernet...");
    
    if (uint8_t mode = Ethernet.begin(mac, 0, ip))
    {
        if (mode == 0x01)
        {
            Serial.print("    IP set Manually: ");
        }
        else if (mode == 0x02)
        {
            Serial.println("    Success using DHCP");
            Serial.print("    IP set via DCHP: ");
            for (byte n = 0; n < 3; n++)
            {
                ipDestination[n] = Ethernet.localIP()[n];
            }
        }
        Serial.println(Ethernet.localIP());
        Serial.print("    Destination address: "); Serial.print(ipDestination);
        Serial.print(":"); Serial.println(portDestination);
        Serial.print("\r\n");
        
        if(Ethernet.startListeningToPort(udpNtrip, AOGNtripPort))
        {
          Serial.print("    Ethernet NTRIP UDP listening to port: ");
          Serial.println(AOGNtripPort);
        }
        if (Ethernet.startListeningToPort(ReceiveUdp, AOGAutoSteerPort))
        {
          Serial.print("    Ethernet AutoSteer UDP listening to port: ");
          Serial.println(AOGAutoSteerPort);
        }
      #ifdef FlasherXOTA
        if (Ethernet.startListeningToPort(ReceiveUDPFlasherX, FlashPort))
        {
          Serial.print("    Ethernet OTA Flash UDP listening to port: ");
          Serial.println(FlashPort);
        }
      #endif
    }
  #endif
}

#ifdef ARDUINO_TEENSY41
void udpNtrip(IPAddress src_ip, uint16_t src_port, uint8_t* udpData, uint16_t len)
{
  SerialGPS->write(udpData, len);
}
#endif


void SendUDPPacket(const uint8_t* buf, uint16_t len)
{
  #ifdef ARDUINO_TEENSY41
    Ethernet.sendCompletePacket((uint8_t)0, buf, len, ipDestination, portDestination);
  #endif
}
