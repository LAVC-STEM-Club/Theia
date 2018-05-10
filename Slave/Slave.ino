#include <SoftwareSerial.h>

#include <Wire.h>
#include "Constants.h"
#include "Adafruit.h"
#include <LiquidCrystal.h>
#include <SPI.h> 


// GLOBAL CONSTANTS IN CONSTANTS.H
// COMPUTATIONS FOR 9-DOF SENSOR ARE IN ACCEL.H  





// speed variables for left and right motors
int leftSpeedHold = 200;
int rightSpeedHold = 200;

// distance detected by ultrasonic sensor
float ultrasonicDistance;  

 

int Contrast=50; 

LiquidCrystal lcd(8, 9, 5, 4, 3, 2); 


//GOAL HEADINGS 
float goalA = 0; 
float goalB = 0; 
float goalC = 0;
float goalD = 0;
float goalE = 0; 
float goalF = 0; 

//REAL TIME HEADING 
int16_t realH = 0; 


//TURNS 
int turnA = 381; 
int turnB = 382; 
int turnC = 383;  
int turnD = 384; 
int turnE = 385;  
int turnF = 386; 

//CORRECTIONS (even values represent deviations to the right and odd values represent deviations to the left)
int a = 2; //Correction for deviation to the right between 1-5 degrees 
int b = 3; //Correction


char buf [100];
volatile byte pos;
volatile bool process_it;


void setup() {
  // open serial connection 
  //Serial.begin(9600); 

  Serial.begin(115200); 

 

  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  pinMode(INPUT3, OUTPUT);
  pinMode(INPUT4, OUTPUT);
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(CAR_LED, OUTPUT);
  pinMode(IR_FRONT_PIN, INPUT);
  pinMode(ECHO,INPUT);
  pinMode(TRIGGER, OUTPUT);

  //initialDelay();
  analogWrite(ENB, leftSpeedHold);
  analogWrite(ENA, rightSpeedHold);
  digitalWrite(BOARD_LED_PIN, LOW);  


  
  analogWrite(6, Contrast);
  lcd.begin(14, 2); 

  filter.begin(50);

  // turn on SPI in slave mode
  SPCR |= bit (SPE);

  // have to send on master in, *slave out*
  pinMode (MISO, OUTPUT);

  // get ready for an interrupt
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();

}




// SPI interrupt routine
ISR (SPI_STC_vect)
{
byte c = SPDR;  // grab byte from SPI Data Register

  // add to buffer if room
  if (pos < sizeof buf)
    {
    buf [pos++] = c;

    // example: newline means time to process buffer
    if (c == '\n')
      process_it = true;

    }  // end of room available
}  // end of interrupt routine SPI_STC_vect


void loop(){   

   if (process_it)
    {
    buf [pos] = 0;
    Serial.println (buf);
    pos = 0;
    process_it = false;
    }  // end of flag set


/*lcd.setCursor(4,0); 
lcd.print("Message");
Serial.println("test"); 

if (process_it)
    {
    buf [pos] = 0;
    lcd.clear();
    lcd.setCursor(4,1); 
    lcd.print(buf);  
    Serial.println(buf); //test    
    pos = 0;
    process_it = false;
    }  // end of flag set

*/

   
    
 


  
 
  
  
  
  
  

  
  /*if (process) {
      process = false; //reset the process
      Serial.println (buff); //print the array on serial monitor 
      lcd.clear(); 
      lcd.setCursor(6,1); 
      lcd.print(buff); 
      indx= 0; //reset button to zero
      
  }*/

  
  
   
  

//Serial.println(x); 


  
  /*byte a=0, b=0;   
  Wire.requestFrom(0x23, 2); 
  a = Wire.read(); 
  b = Wire.read(); 
  x = a; 
  x = x<<8 | b; 
  Serial.print("Value 1: ");
  Serial.println(x); 
  delay(2000); 

  Wire.requestFrom(0x23, 2); 
  a = Wire.read(); 
  b = Wire.read(); 
  x = a; 
  x = x<<8 | b;    
  Serial.print("Value 2: ");
  Serial.println(x); 
  delay(2000); 

  Wire.requestFrom(0x23, 2); 
  a = Wire.read(); 
  b = Wire.read(); 
  x = a; 
  x = x<<8 | b;  
  Serial.print("Value 3: ");
  Serial.println(x); 
  delay(2000);*/      

//x=currentHeading();   
//Serial.println(x); 
//x='\0';  

}



/*void receiveEvent(int howMany){ 

  byte a, b;  
  a = Wire.read(); 
  b = Wire.read();  
  x = a; 
  x = x<<8 | b; 
  
} 

int motorTime(float& a, float& b) { 
 
  int runTime; 

  for (int x=0; x<5; x++){ 
  
    if (b>a+5)
      runTime=10000;  
    else if (b<a-5) 
      runTime=5000; 
    else 
      runTime=7500;
  } 

  return runTime;
  
} 


int motorSpeed(float& a, float& b){

  int leftSpeedHold; 

  for (int x=0; x<5; x++){
     
  
    if (b>a+5) 
      leftSpeedHold=220;     
    else if (b<a-5)  
      leftSpeedHold=180; 
    else 
      leftSpeedHold=200;
  } 
      
  return leftSpeedHold; 
  
} 


/*
Flashes the Arduino Board's LED and causes the program to wait
*/
void initialDelay()
{
  for (int i = 0; i < 5; i++) 
  {
    digitalWrite(BOARD_LED_PIN, LOW);
    delay(1000);
    digitalWrite(BOARD_LED_PIN, HIGH);
    delay(1000);
  }
}


/*
Gets distance reading from ultrasonic sensor
*/




float getDistance() {
  float cm;

  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  cm = pulseIn(ECHO, HIGH);

  // calculate cm and return its value
  return cm / 5.8 / 10;

  /*
  // *debug* display sensor output
  Serial.print(cm);
  Serial.println(" cm");
  delay(500);
  */

}

