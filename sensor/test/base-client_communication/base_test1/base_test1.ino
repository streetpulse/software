//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------

#include <SPI.h>
#include <NRF24.h>


const int INACTIVE_TIME = 1000;
int led = 5;

//-------------------------------------------------------------------------------


unsigned long lastTimeCheckWithServer = 0 ;
#define INTERVAL_BETWEEN_BASE_HEALTH_CHECK 300000 // 5 min = 5 * 60 sec * 1000 millis
#define DATA_PAYLOAD unsigned int


//-------------------------------------------------------------------------
NRF24 nrf24(8,9);

//+++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++
void setup() {
    pinMode(led, OUTPUT);     
    digitalWrite(led, LOW); 
    delay(1000);
    initSensorCommunication();
}


//-------------------------------------------------------------------------
void loop() {

       nrf24.waitAvailableTimeout(100);
       byte data[2];

       uint8_t len            ;
       if (!nrf24.recv((uint8_t*)&data, &len))  ; //  Serial.println(" waiting"); 
       else     digitalWrite(led, HIGH ); 

}


//--------------------------------------------------------------------------
void initSensorCommunication() {
  boolean failedToInitialize = false;
   
  if (!nrf24.init())             {
    failedToInitialize  = true ;
    failTest();
  }
  
  if (!nrf24.setChannel(1))  {
       failTest();
       failedToInitialize  = true ;
  }
  
  if (!nrf24.setThisAddress((uint8_t*)"serv1", 5))     {
       failTest();
       failedToInitialize  = true ;
  }
  
  if (!nrf24.setPayloadSize(sizeof(DATA_PAYLOAD)))    {
       failTest();
       failedToInitialize  = true ;
  }
  
  if (!nrf24.setRF(NRF24::NRF24DataRate2Mbps, NRF24::NRF24TransmitPower0dBm)) {
    failTest();
    failedToInitialize  = true ;
  }
    

  
  
} 


void failTest(){
   digitalWrite(led, HIGH ); 
    delay(1000);
    digitalWrite(led, LOW ); 
    delay(1000);
}
//--------------------------------------------------------------------------
// sends a sign of life to the server
void checkBaseHealthStatus() {
    unsigned long currentTime = millis()  ;
   if (lastTimeCheckWithServer > currentTime) lastTimeCheckWithServer = currentTime - INTERVAL_BETWEEN_BASE_HEALTH_CHECK -1 ; //because millis will go back to 0 after 50 days
   if (currentTime > (lastTimeCheckWithServer + INTERVAL_BETWEEN_BASE_HEALTH_CHECK))    sendHealthToServer();

   
}

//--------------------------------------------------------------------------
void sendHealthToServer() {
  //blink low
}



//--------------------------------------------------------------------------
void sendMessageToServer() {
//blink often long  
}




//------
//------
