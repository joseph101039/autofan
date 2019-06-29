#include "DHT.h"

#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

#define dhtPin 13      //讀取DHT22 Data
#define dhtType DHT22 //選用DHT22   
#define PRINT 0

DHT dht(dhtPin, dhtType); // Initialize DHT sensor
void configure_wdt();
void sleep(int ncycles);
float comfort_level(float T/* Tempture */,float RH/* Humidity */);

// ref: https://folk.uio.no/jeanra/Microelectronics/ArduinoWatchdog.html
// interrupt raised by the watchdog firing
// when the watchdog fires during sleep, this function will be executed
// remember that interrupts are disabled in ISR functions
ISR(WDT_vect)
{
        // not hanging, just waiting
        // reset the watchdog
        wdt_reset();
}

void setup() {
  //to minimize power consumption while sleeping, output pins must not source
    //or sink any current. input pins must have a defined level; a good way to
    //ensure this is to enable the internal pullup resistors.

    for (byte i=0; i<20; i++) {    //make all pins inputs with pullups enabled
        pinMode(i, INPUT_PULLUP);
    }
  
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(dhtPin, INPUT);
  
  #if defined(PRINT) && (PRINT == 1)
  Serial.begin(9600);//設定鮑率9600
  #endif
  dht.begin();//啟動DHT
  configure_wdt();

}

void loop() {
  float h = dht.readHumidity();//讀取濕度
  float t = dht.readTemperature();//讀取攝氏溫度
  
  if (isnan(h) || isnan(t)) {
    Serial.println("無法從DHT傳感器讀取！");
    return;
  }
  int THI = comfort_level(t, h);
  #if defined(PRINT) && (PRINT == 1)
  float f = dht.readTemperature(true);//讀取華氏溫度
  Serial.print("濕度: ");
  Serial.print(h);
  Serial.print("%\t");
  Serial.print("攝氏溫度: ");
  Serial.print(t);
  Serial.print("*C\t");
  Serial.print("華氏溫度: ");
  Serial.print(f);
  Serial.print("*F\t");

  Serial.print("THI: ");
  Serial.print(THI);
  Serial.print("\n");

  if( THI <= 10 ){
    Serial.print("Cold+"); //very cold
  }
  else if( THI <= 15 ){
    Serial.print("Cold "); //0
  }
  else if( THI <= 19 ){
    Serial.print("Cool "); //Slightly cold
  }
  else if( THI <= 26 ){
    Serial.print("Comfo"); //Comfortable
  }
  else if( THI <= 30 ){
    Serial.print("Heat "); //0
  }
  else if( THI >= 31){
    Serial.print("Heat+"); //Heatstroke
  }
  
  #endif
  if(THI > 25)
  {
    //digitalWrite(A1, HIGH);
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
  }
  else{
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
  }

  delay(100);//延時100ms
  sleep(5); //sleep 5 * 8sec(WDT interrupt period)
}



// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution
void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts

  // reminder of the definitions for the time before firing
  // delay interval patterns:
  //  16 ms:     0b000000
  //  500 ms:    0b000101
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001
 
}


// Put the Arduino to deep sleep. Only an interrupt can wake it up.
// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int ncycles)
{  
  int nbr_remaining = ncycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Turn off the ADC while asleep.
  power_adc_disable();
 
  while (nbr_remaining > 0){ // while some cycles left, sleep!

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart
 
  // When awake, disable sleep mode
  sleep_disable();
  
  // we have slept one time more
  nbr_remaining = nbr_remaining - 1;
 
  }
 
  // put everything on again
  power_all_enable();
 
}


//ref: http://work.oknow.org/2018/01/blog-post_29.html
float comfort_level(float T/* Tempture */,float RH/* Humidity */)
{
  float THI;  //Temperature Humidity Index
  float Td;   //Dew point temperature
 
  Td = pow(RH,1/8)*(112+0.9*T)+0.1*T-112;
 
  THI = T-0.55*(1-( exp((17.269*Td)/(Td+237.3)) / exp((17.269*T)/(T+237.3)) ))*(T-14);
//  printf("THI:%f\n",THI);
//  if( THI <= 10 ){
//    sprintf(level,"Cold+"); //very cold
//  }
//  else if( THI <= 15 ){
//    sprintf(level,"Cold "); //0
//  }
//  else if( THI <= 19 ){
//    sprintf(level,"Cool "); //Slightly cold
//  }
//  else if( THI <= 26 ){
//    sprintf(level,"Comfo"); //Comfortable
//  }
//  else if( THI <= 30 ){
//    sprintf(level,"Heat "); //0
//  }
//  else if( THI >= 31){
//    sprintf(level,"Heat+"); //Heatstroke
//  }
 
  return THI;
}
