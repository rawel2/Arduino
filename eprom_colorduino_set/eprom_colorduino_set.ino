

#include <EEPROM.h>


void setup(){
char rob;
rob ='P';
EEPROM.write(0, rob);
rob ='R';
EEPROM.write(1, rob);
rob =10;
EEPROM.write(2, rob);
rob ='P';
EEPROM.write(3, rob);
rob ='R';
EEPROM.write(4, rob);
rob =20;
EEPROM.write(5, rob);
rob =63;
EEPROM.write(6, rob);
rob =50;
EEPROM.write(7, rob);

}

void loop()
{
}




