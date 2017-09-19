// WalkFree
// Copyright 2012  Maen Artimy
//
// Navigate & Avoid Obstacles 
//
 
#include "Ultrasonic.h"
#include <Servo.h> 
#include <Motor.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address 
Ultrasonic ultrasonic(12,13); // 12->trig, 13->echo
Servo myservo;  // create servo object to control a servo 
Motor motor;

const int STEP = 5;
const int SIZE = 37;   // Size of distance array =(180/STEP) +1

int dist[SIZE];
int rDelay;
int mid = 90;          // angle of the forward direction
int s = 16;            // rotation time ms/degree  (16 on carpet and 8 on wooden floor)

void setup() 
{ 
  lcd.begin(20,4);
  myservo.attach(9, 570, 2320);  // attaches the servo on pin 9 to the servo object 
  rDelay = 10 * STEP;            // assuming 10ms/degree speed
  myservo.write(mid);
  
  // ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on   
  
  lcd.setCursor(3,0); //Start at character 4 on line 0
  lcd.print("Hello, world!");
  delay(1000);
  lcd.setCursor(2,1);
  lcd.print("From YourDuino");
  delay(1000);  
  lcd.setCursor(0,2);
  lcd.print("20 by 4 Line Display");
  lcd.setCursor(0,3);
  delay(2000);   
  lcd.cls;
} 

void loop() {
  motor.onFwd(motor.Motor_LR, 100);
  delay(100);
  int distm = ultrasonic.Ranging(CM);
  // if there is an obstacle less than 50cm ahead, stop
  // and find alternate path
  if(distm < 50) { 
      motor.stop(motor.Motor_LR);
      rangeSweep(mid-90, mid+90, dist);
      myservo.write(mid);
      int angle = getAngle(dist);
      
      // if there is a clear path to right or left
      // then move towards it, othrwise, reverse direction
      if(angle > (mid+10)) {
        motor.turnRight(100);
        delay(s * (angle-mid)); // turn
      } else if(angle < (mid-10)) {
        motor.turnLeft(100);
        delay(s * (mid-angle)); // turn
      } else {
        motor.turnRight(100);
        delay(s * 180); // 
      }
  }
 
}

/**
 * Reads distance over 180 degrees twice in left-to-right
 * and right-to-left sweeps then average the readings
 */
void rangeSweep(int st, int en, int dist[]) {
  int pos = 0;    // variable to store the servo position
  for(pos = st; pos<en; pos+=STEP) {
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(rDelay);               // waits 10ms/degree for the servo to reach the position 
    dist[int(pos/STEP)] = ultrasonic.Ranging(CM);
  }

  for(pos = en; pos>st; pos-=STEP) {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(rDelay); 
    dist[int(pos/STEP)] += ultrasonic.Ranging(CM);
    dist[int(pos/STEP)] /= 2;
  }
}

/**
 * Get the angle at which the distance is maximum
 */
int getAngle(int dist[]) {
  int maxDist=0;
  int angle=mid;
  for(int i = 0; i < SIZE; i++) { 
    if(maxDist<dist[i]) {
      maxDist = dist[i];
      angle = i * STEP;
    }
  }
  return angle;
}
