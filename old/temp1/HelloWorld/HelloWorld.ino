/*
  LiquidCrystal Library - Hello World
 
 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.
 
 This sketch prints "Hello World!" to the LCD
 and shows the time.
 
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 
 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */

// include the library code:
#include <LiquidCrystal.h>
#include <dht11.h>
#include <Wire.h>
#include "RTClib.h"

#define DHT11PIN 2

dht11 DHT11;
RTC_DS1307 RTC;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8,9,4,5,6,7);

int p1 = 11;
int p2 = 12;
int led1 = 13;
int hum;
int temp;
int hum1 = 35;
int temp1 = 23;


void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  Wire.begin();
  RTC.begin();
  
  lcd.print("WERSJA: ");
  lcd.print(DHT11LIB_VERSION);
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  pinMode(led1, OUTPUT);
  
  digitalWrite(led1, HIGH);
  digitalWrite(p1, HIGH);
  digitalWrite(p2, HIGH);
  delay(2000);
  digitalWrite(led1, LOW);
  
  if (! RTC.isrunning()) {
    lcd.setCursor(0, 1);
    lcd.print("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  
}  

void loop() {
  int chk = DHT11.read(DHT11PIN);
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  if (chk == 0)
  { 
    hum = DHT11.humidity;
    temp =DHT11.temperature;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hum. (%):");
    lcd.print(hum);
    lcd.setCursor(0, 1);
    lcd.print("Temp.(C):");
    lcd.print(temp);
    if (hum > hum1) 
        {digitalWrite(p1, LOW);
        lcd.setCursor(11, 0);
        lcd.print("(");
        lcd.print(hum1);
        lcd.print(")");
        lcd.print("+");
        }
    else
        {digitalWrite(p1, HIGH);
        lcd.setCursor(11, 0);
        lcd.print("(");
        lcd.print(hum1);
        lcd.print(")");
        lcd.print("-");
        };
    
    if (temp > temp1) 
        {digitalWrite(p2, LOW);
        lcd.setCursor(11, 1);
        lcd.print("(");
        lcd.print(temp1);
        lcd.print(")");
        lcd.print("+");
        }
    else
        {digitalWrite(p2, HIGH);
        lcd.setCursor(11, 1);
        lcd.print("(");
        lcd.print(temp1);
        lcd.print(")");
        lcd.print("-");
        };
    
  }
  else
  { 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Blad odczytu : ");
    lcd.setCursor(0, 1);
    lcd.print(chk);
  }
  
  delay(3000);
  for (int a =1; a < 6; a++){
    DateTime now = RTC.now();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(now.year(), DEC);
    lcd.print('-');
    if (now.month() < 10) {lcd.print('0');}
    lcd.print(now.month(), DEC);
    lcd.print('-');
    if (now.day() < 10) {lcd.print('0');}
    lcd.print(now.day(), DEC);
    lcd.setCursor(0, 1);
    if (now.hour() < 10) {lcd.print('0');}
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    if (now.minute() < 10) {lcd.print('0');}
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    if (now.second() < 10) {lcd.print('0');}
    lcd.print(now.second(), DEC);
    delay(1000);
  }
  
}

