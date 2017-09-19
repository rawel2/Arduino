#include <S65Display.h> 

S65Display lcd; 

word volatile data[88];  //массив входных данных 
byte x=0, y=0, x_end=0, y_end=0; 

void setup()
{   
 TCCR2B = TCCR2B & 0b11111000 | 1;
   pinMode(3,OUTPUT);
   analogWrite(3,137); //включить подсветку
   
  lcd.init(2); 
  lcd.clear(0); 
  Serial.begin(115200); 
} 

  void loop()
{ 
 if (Serial.available() >= 88) 
 { 
   for (byte i = 0; i < 44; i++)
    {
      byte a = Serial.read();
      byte b = Serial.read(); // прочитать данные из порта в массив
     data[i]=((a<<8)|(b)); 
    }  

y_end=y+43;
x_end=x;
if(y_end > 131){y_end=0; x_end++;}
lcd.setArea(x, y, x_end, y_end);
lcd.drawStart();
       for (byte idx = 0; idx < 44; idx++) 
       { 
         lcd.draw(data[idx]); // вывод пикселя на экран 
         y++; 
            if (y > 131) 
             { 
               y = 0; 
               x++; 
             }  
       }  
lcd.drawStop();   
if( x>175 ) x = 0;      
}
}  