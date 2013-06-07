 
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

  // Set up nRF24L01 radio on SPI bus plus pins 3 & 4
  uint8_t cePin = 8              ;
  uint8_t csPin = 9              ;
  RF24 radio(cePin,csPin) ;
  
  const uint64_t baseAddress = 0x6261736531LL ; //0xF0F0F0F0E1LL ;
  #define DATA_PAYLOAD unsigned int // uint8_t
  int led = 5;

//--------------------------------------------------------------------------------------------------
void setup(void)
{
      pinMode(led, OUTPUT);     
      digitalWrite(led, LOW); 
      delay(1000);
    
      radio.begin();
      radio.setPayloadSize(sizeof(DATA_PAYLOAD));
      radio.setRetries(15,15);
       radio.setPALevel(RF24_PA_MAX);
      if(radio.setDataRate(RF24_250KBPS)) {
          successTest(1000);
      }
      radio.setChannel(1);
      radio.setCRCLength(RF24_CRC_16);

      //radio.openWritingPipe(baseAddress);
      radio.openReadingPipe(1, baseAddress);
      radio.openReadingPipe(2, baseAddress);
      radio.openReadingPipe(3, baseAddress);
      radio.openReadingPipe(4, baseAddress);
      radio.openReadingPipe(5, baseAddress);

      radio.startListening() ;
}

//--------------------------------------------------------------------------------------------------
void loop(void) {
     uint8_t  send_payload[2] ;
      send_payload[0] = 0;
      send_payload[1] = 0;      
      
      if ( radio.available() )   {
          radio.read( (uint8_t*)send_payload, sizeof(DATA_PAYLOAD) );
        if (send_payload[1] != 0) {
          successTest(200);
           successTest(200);
            successTest(200);
        }
        else  successTest(1000);
      }
}

//--------------------------------------------------------------------------------------------------
void successTest(int time){
   digitalWrite(led, HIGH ); 
    delay(time);
    digitalWrite(led, LOW ); 
    delay(time);
}
