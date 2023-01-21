//******************************************************************************
// FlasherX -- firmware "OTA" update via Intel Hex file
//******************************************************************************

#ifdef FlasherXOTA
#ifdef ARDUINO_TEENSY41
  #include "zFlashTxx.h"


//******************************************************************************
// hex_info_t struct for hex record and hex file info
//******************************************************************************
typedef struct {  //
  uint8_t *data;   // pointer to array allocated elsewhere
  unsigned int addr;  // address in intel hex record
  unsigned int code;  // intel hex record type (0=data, etc.)
  unsigned int num; // number of data bytes in intel hex record

  uint32_t base;  // base address to be added to intel hex 16-bit addr
  uint32_t min;   // min address in hex file
  uint32_t max;   // max address in hex file

  int eof;    // set true on intel hex EOF (code = 1)
  unsigned int lines;    // number of hex records received
} hex_info_t;

uint32_t buffer_addr, buffer_size;
bool updatemode = false;
static uint8_t data[16];// buffer for hex data

hex_info_t hex =
{ // intel hex info struct
  data, 0, 0, 0,          //   data,addr,num,code
  0, 0xFFFFFFFF, 0,           //   base,min,max,
  0, 0            //   eof,lines
};

byte OTAUpdate[9] = {0x4f, 0x54, 0x41, 0x55, 0x70, 0x64, 0x61, 0x74, 0x65};





void ReceiveUDPFlasherX(IPAddress src_ip, uint16_t src_port, uint8_t* udpData, uint16_t len)
{
    if (updatemode)
    {
      // read and process intel hex lines until EOF or error
      if (process_hex_record( udpData, len ))
      {
        
        uint8_t failOTA[6] = {0x52, 0x45, 0x42, 0x4f, 0x4f, 0x54};
        SendUDPPacket(failOTA, 6);
        
        // return error or user abort, so clean up and
        // reboot to ensure that static vars get boot-up initialized before retry
        Serial.println();
        Serial.printf( "erase FLASH buffer / free RAM buffer...\n" );
        delay(5000);
        firmware_buffer_free( buffer_addr, buffer_size );
        REBOOT;
      }
    }
    else if (len == 9)
    {
      bool startOTA = false;
      for (byte idx = 0; idx < len && idx < 9;)
      {
        if (udpData[idx] == OTAUpdate[idx])
        {
          startOTA = true;
          idx+=1;
        }
        else
        {
          startOTA = false;
          break;
        }
      }
      
      if(startOTA)
      {
        if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0)
        {
          Serial.println();
          Serial.printf( "unable to create buffer\n" );
          Serial.flush();
          for (;;) {}
        }
        
        Serial.printf( "target = %s (%dK flash in %dK sectors)\n", FLASH_ID, FLASH_SIZE/1024, FLASH_SECTOR_SIZE/1024);
        Serial.printf( "buffer = %1luK %s (%08lX - %08lX)\n", buffer_size / 1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM", buffer_addr, buffer_addr + buffer_size );
        Serial.println( "waiting for hex lines...\n" );
        updatemode = true;
      }
    }
}
    
void SENDCheckUdp()
{
  uint8_t checkOTA[9] = {0x4f, 0x54, 0x41, 0x55, 0x70, (uint8_t)(hex.lines), (uint8_t)(hex.lines>>8), (uint8_t)(hex.lines>>16), (uint8_t)(hex.lines>>24)};
  SendUDPPacket(checkOTA, 9);
}

//******************************************************************************
// process_hex_record()    process record and return okay (0) or error (1)
//******************************************************************************
int process_hex_record(uint8_t *packetBuffer, int packetSize)
{
  if (packetSize < 5)
  {
    //firmware_buffer_free( FLASH_BASE_ADDR, FLASH_SIZE - FLASH_RESERVE);
    //REBOOT;
    
    return 1;
  }
  else if (packetBuffer[0] != 0x3a)
  {
    Serial.printf( "abort - invalid hex code %d\n", hex.code );
    
    return 1;
  }
  else
  {
    for (byte idx = 1; idx + 4 < packetSize;)
    {
      byte len = packetBuffer[idx];
      unsigned int addr = packetBuffer[idx + 1] << 8 | packetBuffer[idx + 2];

      byte type = packetBuffer[idx + 3];
      if (idx + 4 + len < packetSize)
      {
        unsigned sum = (len & 255) + ((addr >> 8) & 255) + (addr & 255) + (type & 255);

        for (byte j = 0; j < len; j++)
        {
          sum += packetBuffer[idx + 4 + j] & 255;
          hex.data[j] = packetBuffer[idx + 4 + j];
        }
        hex.num = len;
        hex.code = type;
        hex.addr = addr;
        
        byte Checksum = packetBuffer[idx + 4 + len];
        if (((sum & 255) + (Checksum & 255)) & 255)
        {
          Serial.println("abort - bad hex line");
          return 1;
        }
        else
        {
          if (hex.code == 0)// if data record
          {
            // update min/max address so far
            if (hex.base + hex.addr + hex.num > hex.max)
              hex.max = hex.base + hex.addr + hex.num;
            if (hex.base + hex.addr < hex.min)
              hex.min = hex.base + hex.addr;

            uint32_t addr = buffer_addr + hex.base + hex.addr - FLASH_BASE_ADDR;
            if (hex.max > (FLASH_BASE_ADDR + buffer_size))
            {
              Serial.printf( "abort - max address %08lX too large\n", hex.max );
              return 1;
            }
            else if (!IN_FLASH(buffer_addr))
            {
              memcpy( (void*)addr, (void*)hex.data, hex.num );
            }
            else if (IN_FLASH(buffer_addr))
            {
              int error = flash_write_block( addr, hex.data, hex.num );
              if (error)
              {
                Serial.println();
                Serial.printf( "abort - error %02X in flash_write_block()\n", error );
                return 1;
              }
            }
          }
          else if (hex.code == 1)
          {
            // EOF (:flash command not received yet)
            Serial.println( "EOF");
            hex.eof = 1;
          }
          else if (hex.code == 2)
          {
            // extended segment address (top 16 of 24-bit addr)
            hex.base = ((hex.data[0] << 8) | hex.data[1]) << 4;
          }
          else if (hex.code == 4)
          { // extended linear address (top 16 of 32-bit addr)
            hex.base = ((hex.data[0] << 8) | hex.data[1]) << 16;
          }
          else if (hex.code == 5)
          { // start linear address (32-bit big endian addr)
            hex.base = (hex.data[0] << 24) | (hex.data[1] << 16)
                       | (hex.data[2] <<  8) | (hex.data[3] <<  0);
          }
          else if (hex.code == 6)
          {
              Serial.printf( "UPDATE!!!\n" );
              // move new program from buffer to flash, free buffer, and reboot

              //flash_move( FLASH_BASE_ADDR, buffer_addr, hex.max-hex.min );

              return 1;
          }
          else if (hex.code == 7)
          {
            Serial.printf( "LINES DONT Match\n" );
            return 1;
          }
          else
          {
            // error on bad hex code
            Serial.println();
            Serial.printf( "abort - invalid hex code %d\n", hex.code );
            return 1;
          }

          hex.lines++;
          //Serial.println(hex.lines);

          if (hex.eof)
          {
            Serial.println();
            Serial.printf( "\nhex file: %1d lines %1lu bytes (%08lX - %08lX)\n", hex.lines, hex.max - hex.min, hex.min, hex.max );

            // check FSEC value in new code -- abort if incorrect
            
          #if defined(KINETISK) || defined(KINETISL)
            uint32_t value = *(uint32_t *)(0x40C + buffer_addr);
            if (value == 0xfffff9de)
            {
              Serial.println();
              Serial.printf( "new code contains correct FSEC value %08lX\n", value );
            }
            else
            {
              Serial.println();
              Serial.printf( "abort - FSEC value %08lX should be FFFFF9DE\n", value );
              return 1;
            }
          #endif

            // check FLASH_ID in new code - abort if not found
            if (check_flash_id( buffer_addr, hex.max - hex.min ))
            {
              Serial.println();
              Serial.printf( "new code contains correct target ID %s\n", FLASH_ID );
              SENDCheckUdp();
            }
            else
            {
              Serial.println();
              Serial.printf( "abort - new code missing string %s\n", FLASH_ID );
              return 1;
            }
          }
          idx += 6 + len;//need for extra ::
        }
      }
      else
      {
        Serial.printf( "abort - invalid hex code %d\n", hex.code );
        return 1;
      }
    }
  }
  return 0;
}
#endif
#endif
