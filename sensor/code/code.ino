//----------------------------------------------------------------
// Street Pulse Sensor
//----------------------------------------------------------------
/*   
 Hardware setup:
 MMA8452 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 INT2 ---------------------- D3
 INT1 ---------------------- D2
 GND ---------------------- GND
 
 NRF Hardware SPI:
 MISO ->  D12 purple
 MOSI ->  D11 blue
 SCK  ->   D13 grey
 CSN  ->  D10 brown
 CE    ->     D8 green (chip is active if CE is high)
 POWR -> D9 red
 
 */


//----------------------------------------------------------------
#include "i2c.h"  // not the wire library, can't use pull-ups
#include <avr/sleep.h>
#include <avr/wdt.h> 
#include <avr/power.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


#define SENSOR_ADDRESS 2 

//-----------------------------------------------------------------
#define digitalPin0   0
#define digitalPin1   1
#define digitalPin4   4
#define digitalPin5   5
#define digitalPin6   6
#define digitalPin7   7
#define nrfPower     9        

#define CE_PIN      8
#define CS_PIN      10
#define BASE_ADDRESS 0x6261736531LL

#define  WHO_AM_I                    0x0D 
#define  ACCELERATOR_NAME            0x2A
#define  FIRST_ACCELERATOR_ADDRESS   0x1C
#define  SECOND_ACCELERATOR_ADDRESS  0x1C

#define SERIAL_SPEED 115200

#define TWO_G      0
#define FOUR_G     1
#define HEIGHT_G   2

#define ACCELERO_STANDBY  0xFE 
#define ACCELERO_ACTIVE   0x01


//------------------------------------------------------------------
#define CTRL_REG2  0x2B
#define NO_SLEEP   0x00
#define AUTO_SLEEP 0x04

#define SLEEP_NORMAL_POWER_MODE     0x00 // bit 3 and 4 
#define SLEEP_LOW_POWER_MODE        0x08 // bit 3 and 4 
#define SLEEP_MIN_POWER_MODE        0x18 // bit 3 and 4 

#define WAKE_NORMAL_SAMPLING_MODE   0x00 // bit 0 and 1 
#define WAKE_LOW_SAMPLING_MODE      0x3 // bit 0 and 1


//------------------------------------------------------------------
#define CTRL_REG1                   0x2A
#define CLEAR_SAMPLE_RATE_SETTING   0x06
#define SLEEP_MODE_SAMPLE_RATE_12Hz 0x40 //bit 7 and 6
#define SLEEP_MODE_SAMPLE_RATE_50Hz 0x00 
#define WAKE_MODE_SAMPLE_RATE_12Hz  0x28
#define WAKE_MODE_SAMPLE_RATE_800Hz  0

#define DEEP_SLEEP_MODE    0b000000011 
#define SLEEP_MODE_ON        0x04
#define SLEEP_MODE_OFF      0xFB      

#define SLEEP_SAMPLING_SLOW 0b01000000
#define WAKE_SAMPLING_SLOW  0b00101000

#define GO_TO_SLEEP_TIMEOUT         0x29
#define SLEEP_IMMEDIATELY             0
#define SLEEP_AFTER_10_SEC_AT_12Hz   31    
#define SLEEP_AFTER_5_SEC_AT_12Hz    15    
#define SLEEP_AFTER_5_SEC_AT_800Hz   15


#define XYZ_DATA_CFG             0x0E
#define HEIGH_HUNDRED_HZ 0xC7
#define TWELVE_HZ                   0xEF

//------------------------------------------------------------------
#define INTERRUPT_WAKE         0x2C
#define INTERRUPT_ACTIVE_HIGH  0x02
#define WAKE_ON_TRANSIENT      0x40 //bit 6
#define WAKE_ON_FREE_FALL      0x08  //bit 3
#define WAKE_ON_PULSE          0x10
#define WAKE_ON_MOTION         0x08 //same as Free fall here
#define WAKE_ON_ALL            0x78

//------------------------------------------------------------------
#define INTERRUPT_ENABLED      0x2D
#define INTERRUPT_DATA_READY   0x01
#define INTERRUPT_MOTION       0x04
#define INTERRUPT_PULSE        0x08
#define INTERRUPT_ORIENTATION  0x10
#define INTERRUPT_TRANSIENT    0x20
#define INTERRUPT_SLEEP_MODE   0x80

//------------------------------------------------------------------
#define INTERRUPT_PIN_ASSIGNMENT 0x2E
#define DATA_READY_PIN1        0x01
#define MOTION_ON_PIN1         0x04
#define PULSE_ON_PIN1          0x08
#define ROTATE_ON_PIN1         0x10
#define TRANSIENT_ON_PIN1      0x20
#define SLEEP_ON_PIN1          0x80

//------------------------------------------------------------------
#define PULSE_CONFIGURATION           0x21
#define PULSE_TIME_WINDOW               0x26
#define INTERVAL_BETWEEN_PULSES 0x27
#define PULSE_SINGLE_TAP_X_Y_Z      0x15
#define PULSE_THRESHOLD_X               0x23
#define PULSE_THRESHOLD_Y               0x24
#define PULSE_THRESHOLD_Z               0x25
#define FLAG_PULSES                              0x40  // Pulses occurence are kept in a variable until that variable is read

//------------------------------------------------------------------
#define MOTION_CONFIGURATION 0x15
#define FREE_FALL_DETECTOR      0x00 //carefull, this needs an NAND to function 
#define MOTION_DETECTOR            0x40
#define X_AXIS_ENABLED                 0x08
#define Y_AXIS_ENABLED                 0x10
#define Z_AXIS_ENABLED                 0x20
#define FLAG_MOTIONS                    0x80  // Motion occurence are kept in a variable until that variable is read

//------------------------------------------------------------------
#define MOTION_THRESHOLD         0x17  //Range is from 0 to 127
#define THRESHOLD_ZERO            0x02
#define THRESHOLD_TINY            0x04
#define THRESHOLD_MINI            0x06
#define THRESHOLD_MINI_SMALL      0xA
#define THRESHOLD_SMALL           0xC
#define THRESHOLD_MEDIUM          0x14
#define THRESHOLD_BIG             0x20
#define THRESHOLD_HUGHE           0x30

#define MOTION_SAMPLE 0x18 // 8 bit value for window of samples to look at for motion detection
#define MINIMUM_SAMPLE   1
#define ZERO_SAMPLE      2
#define ONE_SAMPLE       3
#define TINY_SAMPLE      4
#define CUTE_SAMPLE      5
#define MINI_SAMPLE      8
#define SMALL_SAMPLE    16
#define MEDIUM_SAMPLE   32
#define LARGE_SAMPLE     64
#define BIG_SAMPLE         128
#define MAX_SAMPLE        255

#define START_ACCELERATOR_FAILED 99999
#define START_SENSOR_OK          00000
#define COVER_FLIPPED            99998
#define COVER_OPEN               60000


#define DATA_PAYLOAD unsigned int
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int firstInterruptPin       = 2 ;  
int secondInterruptPin = 3 ;
const byte dataRate    = 0 ;  // output data rate: 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
const uint64_t baseAddress = BASE_ADDRESS ;

int accelCount[6]              ;  // Stores the 12-bit signed value

boolean firstAcceleratorPlugged       = false ;
boolean secondAcceleratorPlugged = false ;
boolean knownProblemWithAccelerometer = false ;
volatile boolean binCoverMoves        = false ;
volatile boolean isSendingToServer   = false ;

unsigned long lastInterrupt = millis();

// --------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------
void setup() {

  wdt_disable() ;                               // turn off watchdog to save energy
  power_timer2_disable() ; 
  power_timer1_disable() ;  
  power_usart0_disable() ;

  power_twi_disable(); //I2C com to accelerometer OFF
  power_spi_disable(); // SPI com to wireless module OFF

  // SERIAL PORT PINS MUST BE NOT SET MANUALLY
  pinMode      (A0,INPUT); digitalWrite(A0, HIGH);  // set pullup on analog pin 0 
  pinMode      (A1,INPUT); digitalWrite(A1, HIGH);  // set pullup on analog pin 0 
  pinMode      (A2,INPUT); digitalWrite(A2, HIGH);  // set pullup on analog pin 0 
  pinMode      (A3,INPUT); digitalWrite(A3, HIGH);  // set pullup on analog pin 0 
  pinMode      (A4,INPUT); digitalWrite(A4, HIGH);  // set pullup on analog pin 0 
  pinMode      (A5,INPUT); digitalWrite(A5, HIGH);  // set pullup on analog pin 0 
  pinMode      (A6,INPUT); digitalWrite(A6, HIGH);  // set pullup on analog pin 0 
  pinMode      (A7,INPUT); digitalWrite(A7, HIGH);  // set pullup on analog pin 0 
  
  
  pinMode      (digitalPin4,INPUT);   digitalWrite(digitalPin4, HIGH);       // turn on pullup resistors to save energy
  pinMode      (digitalPin5,INPUT);    digitalWrite(digitalPin5, HIGH);       // turn on pullup resistors to save energy
  pinMode      (digitalPin6,INPUT);    digitalWrite(digitalPin6, HIGH);       // turn on pullup resistors to save energy
  pinMode      (digitalPin7,INPUT);    digitalWrite(digitalPin7, HIGH);       // turn on pullup resistors to save energy

  pinMode      (nrfPower , OUTPUT) ;   digitalWrite  (nrfPower  , LOW   ) ;
  pinMode      (firstInterruptPin  , INPUT    ) ;    // Set up the interrupt pins, active high, push-pull 
  digitalWrite  (firstInterruptPin  , LOW       ) ;
  pinMode      (secondInterruptPin , INPUT ) ;  digitalWrite  (secondInterruptPin, HIGH    )  ;

  setupAccelerometer(TWO_G, 0,FIRST_ACCELERATOR_ADDRESS);  // init the accelerometer 
  sendDataToServer(1);      

}


// --------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------
void loop() { 

  if  ( !isSendingToServer && binCoverMoves  )   {
    isSendingToServer = true    ;
    sendDataToServer(1); 
    attachInterrupt(0, motionDetected, CHANGE);
  }

  sleepSensor();

}


// --------------------------------------------------------------------------------------------------------------
void motionDetected(){
  if ( isSendingToServer ) return;
  sleep_disable();
  detachInterrupt(0);
  binCoverMoves = true;
}   

// --------------------------------------------------------------------------------------------------------------
void sleepSensor(){
  attachInterrupt(0, motionDetected, CHANGE);


  ADCSRA = 0;
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins    
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sei();
  // turn off brown-out enable in software
  MCUCR = _BV (BODS) | _BV (BODSE);  // turn on brown-out enable select
  MCUCR = _BV (BODS);        // this must be done within 4 clock cycles of above
  sleep_cpu();
  sleep_disable();   

}


// --------------------------------------------------------------------------------------------------------------
void sendDataToServer (unsigned int data)  {

  power_spi_enable() ;
  digitalWrite(nrfPower,HIGH);  

  lastInterrupt         = millis() ;
  binCoverMoves = false    ;
  byte sendingBuffer[2];

  //shit bat crazy: flip bits if notation is negative number an place them in right place
  boolean isNegative = (data > 32767);
  if (isNegative) data = ~data;

  data = (data << 7) ;
  data += SENSOR_ADDRESS ;
  if (isNegative) data += 32768 ; // sign bit
  sendingBuffer[0] = data  >> 8    ;           
  sendingBuffer[1] = data &  255  ;



  RF24 radio(CE_PIN,CS_PIN) ;             

  radio.begin();
  radio.setPayloadSize(sizeof(DATA_PAYLOAD));
  radio.setRetries(15,15);
  radio.setPALevel(RF24_PA_MAX);

  radio.setDataRate(RF24_250KBPS) ;            
  radio.setChannel(1);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(baseAddress);
  radio.startListening() ;
  radio.stopListening();                        // First, stop listening so we can talk      
  radio.write( sendingBuffer , radio.getPayloadSize() );

  radio.powerDown(); 
  isSendingToServer = false    ;
  power_spi_disable();

}        



// --------------------------------------------------------------------------------------------------------------
void readAccelData(int * destination, unsigned char accelerometer) {
  byte rawData[6];  // x/y/z accel register data stored here

  power_twi_enable();
  readRegisters(0x01, 6, &rawData[0],accelerometer);  // Read the six raw data registers into data array
  power_twi_disable();

  for (int i=0; i<6; i+=2)  {  // loop to calculate 12-bit ADC and g value for each axis 
    destination[i/2] = ((rawData[i] << 8) | rawData[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
    if (rawData[i] > 0x7F)  {  // If the number is negative, we have to make it so manually (no 12-bit data type)
      destination[i/2] = ~destination[i/2] + 1;
      destination[i/2] *= -1;  // Transform into negative 2's complement #
    }
  }
}



// --------------------------------------------------------------------------------------------------
void setupAccelerometer(byte gResolution, byte dataRate, unsigned char accelerometer)
{

  power_twi_enable();
  setAcceleroToStandby(accelerometer);                         // Must be in standby to change registers
  writeRegister(XYZ_DATA_CFG,   gResolution, accelerometer);   // set to 2G
  writeRegister(CTRL_REG1,     SLEEP_MODE_SAMPLE_RATE_12Hz | WAKE_MODE_SAMPLE_RATE_12Hz          , accelerometer);  
  writeRegister(CTRL_REG2,     SLEEP_MIN_POWER_MODE        | WAKE_LOW_SAMPLING_MODE | AUTO_SLEEP  , accelerometer);    

  //MOTION DETECTOR
  writeRegister(MOTION_CONFIGURATION     ,  MOTION_DETECTOR | Z_AXIS_ENABLED       , accelerometer);  // multiply by 0.0625g/LSB to get the threshold
  writeRegister(MOTION_THRESHOLD         ,  THRESHOLD_MINI_SMALL                        , accelerometer);
  writeRegister(MOTION_SAMPLE            ,  MINIMUM_SAMPLE                            , accelerometer);  

  //SETTING UP INTERRUPT
  writeRegister(INTERRUPT_WAKE           ,  INTERRUPT_ACTIVE_HIGH | WAKE_ON_MOTION , accelerometer);  // Active high, push-pull interrupts
  writeRegister(INTERRUPT_ENABLED        ,  INTERRUPT_MOTION                       , accelerometer);  
  writeRegister(INTERRUPT_PIN_ASSIGNMENT ,  MOTION_ON_PIN1                         , accelerometer);  // DRDY on INT1, P/L and taps on INT2
  writeRegister(GO_TO_SLEEP_TIMEOUT       ,  SLEEP_IMMEDIATELY                      , accelerometer);

  /*Serial.print( "CTRL_REG1  -SLEEP/WAKE SAMPLING RATE: ");       Serial.println(  readRegister(CTRL_REG1,accelerometer));   
   Serial.print( "CTRL_REG2  -SLEEP_MODE_ON: ");                             Serial.println( readRegister(CTRL_REG2       ,accelerometer));   
   Serial.print( "TIMEOUT TO SLEEP: time to wait before sleep: ");          Serial.println(  readRegister(GO_TO_SLEEP_TIMEOUT,accelerometer));   
   Serial.print( "INTERRUPT_WAKE  -interrupt setting: ");                         Serial.println(  readRegister(INTERRUPT_WAKE,accelerometer) );   
   Serial.print( "INTERRUPT ENABLED  -interrupt setting: ");                   Serial.println( readRegister(INTERRUPT_ENABLED,accelerometer) );   
   Serial.print( "INTERRUPT_PIN_ASSIGNMENT  -interrupt setting: ");   Serial.println(  readRegister(INTERRUPT_PIN_ASSIGNMENT,accelerometer) );   
   Serial.print( "ACCELERO STATUS: ");                                                   Serial.println(   readRegister(0x0B ,  accelerometer));
   */
  setAcceleroToActive(accelerometer);  // Set to active to start reading

  power_twi_disable();

}


// -------------------------------------------------------------------------------------------------
// Sets the MMA8452 to standby mode. It must be in standby to change most register settings 
void setAcceleroToStandby(unsigned char accelerometer) {
  writeRegister( CTRL_REG1, readRegister( CTRL_REG1, accelerometer ) & ACCELERO_STANDBY, accelerometer );
}

// -------------------------------------------------------------------------------------------------
// Sets the MMA8452 to active mode;   Needs to be in this mode to output data
void setAcceleroToActive(unsigned char accelerometer) {
  byte c = readRegister( CTRL_REG1,                                            accelerometer  );
  writeRegister( CTRL_REG1, c | ACCELERO_ACTIVE, accelerometer );
}

// -------------------------------------------------------------------------------------------------
// Read i registers sequentially, starting at address  into the dest byte array
void readRegisters(byte address, int i, byte * dest, unsigned char accelerometer)
{
  i2cSendStart();                                  
  i2cWaitForComplete();
  i2cSendByte((accelerometer<<1));  
  i2cWaitForComplete(); 	// write 0xB4
  i2cSendByte(address);                     
  i2cWaitForComplete(); 	// write register address

  i2cSendStart();
  i2cSendByte((accelerometer<<1)|0x01);  
  i2cWaitForComplete();// write 0xB5

  for (int j=0; j<i; j++)  {
    i2cReceiveByte(TRUE);    
    i2cWaitForComplete();
    dest[j] = i2cGetReceivedByte();	// Get MSB result
  }
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN);	// Disable TWI
  sbi(TWCR, TWEN);	// Enable TWI
}

// -------------------------------------------------------------------------------------------------
/* read a single byte from address and return it as a byte */
byte readRegister(uint8_t address, unsigned char accelerometer)
{
  byte data;

  i2cSendStart();                                  
  i2cWaitForComplete();
  i2cSendByte((accelerometer<<1));  
  i2cWaitForComplete();	// write 0xB4
  i2cSendByte(address);	                     
  i2cWaitForComplete();// write register address

  i2cSendStart();

  i2cSendByte((accelerometer<<1)|0x01);  
  i2cWaitForComplete(); // write 0xB5
  i2cReceiveByte(TRUE);                            
  i2cWaitForComplete();

  data = i2cGetReceivedByte();	              
  i2cWaitForComplete();// Get MSB result
  i2cSendStop();

  cbi(TWCR, TWEN);	// Disable TWI
  sbi(TWCR, TWEN);	// Enable TWI

  return data;
}

//----------------------------------------------------------------------------------------------------------------------
/* Writes a single byte (data) into address */
void writeRegister(unsigned char address, unsigned char data, unsigned char accelerometer)
{
  i2cSendStart();                                  
  i2cWaitForComplete();
  i2cSendByte((accelerometer<<1));  
  i2cWaitForComplete(); // write 0xB4
  i2cSendByte(address);	                    
  i2cWaitForComplete(); // write register address
  i2cSendByte(data);                           
  i2cWaitForComplete();
  i2cSendStop();
}

