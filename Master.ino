

#include <SoftwareSerial.h>

//MASTER BOARD
//Runs 9Dof Sensor and sends deviations to the slave

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Madgwick.h>
#include <Mahony.h>
#include <Wire.h>
#include "Constants.h"
#include "Accel.h"
#include <LiquidCrystal.h>
#include <SPI.h>


int x = 0; //Real time heading

int Contrast = 100;

LiquidCrystal lcd(8, 9, 5, 4, 3, 2);

void setup() {
  // open serial connection
  Serial.begin(115200);

  // Initialize the sensors.
  // Accel.h
  initSensors();

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
  //pinMode(ECHO,INPUT);
  pinMode(TRIGGER, OUTPUT);

  initialDelay();
  analogWrite(ENB, leftSpeedHold);
  analogWrite(ENA, rightSpeedHold);
  digitalWrite(BOARD_LED_PIN, LOW);


  //Sets board as master
  Wire.begin();

  analogWrite(6, Contrast);
  lcd.begin(14, 2);



  filter.begin(50);

  digitalWrite(SS, HIGH);  // ensure SS stays high for now

  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}

void loop()
{

  char c;

  // enable Slave Select
  digitalWrite(SS, LOW);    // SS is pin 10

  // send test string
  for (const char * p = "Hello, world!\n" ; c = *p; p++)
    SPI.transfer (c);

  // disable Slave Select
  digitalWrite(SS, HIGH);

  delay (1000);  // 1 seconds delay


  /*char c;

  for (int i = 0; i <= 100; i++) {
    x = heading();
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Heading");
    lcd.setCursor(6, 1);
    lcd.print(x);
    Serial.println(x);
    // enable Slave Select
    digitalWrite(SS, LOW);    // SS is pin 10
    // send test string
    for ( const char* p = "Puta\n" ; c = *p; p++)
      SPI.transfer (c);
    // disable Slave Select
    digitalWrite(SS, HIGH);
  }

  for (int i = 0; i <= 100; i++) {
    x = heading();
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Heading");
    lcd.setCursor(6, 1);
    lcd.print(x);
    Serial.println(x);
    // enable Slave Select
    digitalWrite(SS, LOW);    // SS is pin 10
    // send test string
    for ( const char* m = "Biatch\n" ; c = *m; m++)
      SPI.transfer (c);
    // disable Slave Select
    digitalWrite(SS, HIGH);
  }*/






  /*int count = 0;

    x=heading();
    Serial.println(x);
    count++;
    mySerial.print(x);*/

    

  /*int count = 0; //Counts loops in for loop
    int16_t a = 0; //Variable that holds heading value being sent to slave


    //1st STRAIGHT AWAY (goalA)
    for(int i=1; i<=1000; i++){  //Change i depending on distance of path
    x=heading();
    Serial.println(x);
    count++;
    if (count%10 == 0) {
      Wire.beginTransmission(0x23);
      int16_t a = x;
      magArray[0] = (a>>8) & 0xFF;
      magArray[1] = a & 0xFF;
      Wire.write(magArray, 2);
      Wire.endTransmission();
    }
    }

    //1st TURN (turnA)
    x = 38100000;
    Serial.println(x);
    delay(5000);
    Wire.beginTransmission(0x23);
    a = x;
    magArray[0] = (a>>8) & 0xFF;
    magArray[1] = a & 0xFF;
    Wire.write(magArray, 2);
    Wire.endTransmission();

    count = 0; //Resets count for next for loop

    //2nd STRAIGHT AWAY
    for(int i=1; i<=100; i++){
    x=heading();
    Serial.println(x);
    count++;
    if (count%10 == 0) {
      Wire.beginTransmission(0x23);
      a = x;
      magArray[0] = (a>>8) & 0xFF;
      magArray[1] = a & 0xFF;
      Wire.write(magArray, 2);
      Wire.endTransmission();
    }
    }



    //2nd TURN (turnB)
    x = 38200000;
    Serial.println(x);
    delay(5000);
    Wire.beginTransmission(0x23);
    a = x;
    magArray[0] = (a>>8) & 0xFF;
    magArray[1] = a & 0xFF;
    Wire.write(magArray, 2);
    Wire.endTransmission();*/
}

/*int16_t spiWriteAndRead(int16_t dout){
  int16_t din = 0;
  digitalWrite(pinSS, LOW);
  delay(1);
  din = SPI.transfer(dout);
  digitalWrite(pinSS, HIGH);
  return din;
  }*/


void test() {

  x = heading(); 
  Serial.println(x);
}


float heading()
{

  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  float gx = gyro_event.gyro.x * 57.2958F;
  float gy = gyro_event.gyro.y * 57.2958F;
  float gz = gyro_event.gyro.z * 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  // Print the orientation filter output
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();
  

  delay(10);

  return heading;
}







