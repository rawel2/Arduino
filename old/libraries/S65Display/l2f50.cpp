extern "C"
{
  #include <inttypes.h>
  #include <avr/io.h>
  #include <util/delay.h>
}
//#include "wiring.h"
#include "Arduino.h"
#include "S65Display.h"
#include "l2f50.h"

void s65_drawStart(void)
{

//s65_writeCmd(0xAE); //display off
  s65_writeCmd(0x5C);// Display Memory write
  S65_CS_ENABLE();

  return;
}


void s65_draw(uint16_t color)
{
 // s65_writeSPI(color>>8);
 // s65_writeSPI(color);
  SPDR = (color>>8);  
  while(!(SPSR & _BV(SPIF)));
  SPDR = (color);  
  while(!(SPSR & _BV(SPIF)));

  return;
}

void s65_drawStop(void)
{
//s65_writeCmd(0xAF); //display on
  S65_CS_DISABLE();

  return;
}

void s65_setArea(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  s65_writeCmd(0x15);                    //column address set 
  s65_writeData(0x08+y0);                //start column
  s65_writeData(0x01);                   //start column
  s65_writeData(0x08+y1);                //end column
  s65_writeData(0x01);                   //end column

  s65_writeCmd(0x75);                    //page address set 
  s65_writeData(x0);                     //start page, начало по оси х
  s65_writeData(x1);                     //end page, конец по оси х

  return;
}

void s65_setCursor(uint8_t x, uint8_t y)
{
  s65_setArea(x, y, x, y);

  return;
}

void s65_init(void)
{
uint8_t i;
static const uint8_t gcp64_0[29] PROGMEM =
                      {0x11,0x27,0x3C,0x4C,0x5D,0x6C,0x78,0x84,0x90,0x99,0xA2,0xAA,0xB2,0xBA,
                       0xC0,0xC7,0xCC,0xD2,0xD7,0xDC,0xE0,0xE4,0xE8,0xED,0xF0,0xF4,0xF7,0xFB,
                       0xFE};
  static const uint8_t gcp64_1[34] PROGMEM =
                     {0x01,0x03,0x06,0x09,0x0B,0x0E,0x10,0x13,0x15,0x17,0x19,0x1C,0x1E,0x20,
                      0x22,0x24,0x26,0x28,0x2A,0x2C,0x2D,0x2F,0x31,0x33,0x35,0x37,0x39,0x3B,
                      0x3D,0x3F,0x42,0x44,0x47,0x5E};
  static const uint8_t gcp16[15] PROGMEM =
                      {0x13,0x23,0x2D,0x33,0x38,0x3C,0x40,0x43,0x46,0x48,0x4A,0x4C,0x4E,0x50,0x64};

  //------------reset-------------
  S65_RST_ENABLE(); //LOW // reset display
  S65_CS_DISABLE(); //HIGH  // CS is high during reset release
  S65_RS_DISABLE(); //HIGH   // RS is set to high
  _delay_ms(10);
  S65_RST_DISABLE(); //HIGH  // release reset
  _delay_ms(35);
  S65_CS_ENABLE(); //LOW   // select display

  //s65_writeCmd(0xAE); //display off

  s65_writeCmd(0xBC);  //DATCTL Data Control (data handling in RAM)

  s65_writeData(0x2B); //565 mode, 0x2A=normal
			//565 mode, 0x2B=MIRROR
			// 0x0A=666mode, 0x3A=444mode

  S65_CS_DISABLE(); //HIGH
  asm volatile("nop");
  S65_CS_ENABLE(); //LOW

  s65_writeCmd(0xCA);  //display control 
  s65_writeData(0x4C); //P1
  s65_writeData(0x01); //P2
  s65_writeData(0x53); //P3
  s65_writeData(0x00); //P4
  s65_writeData(0x02); //P5
  s65_writeData(0xB4); //P6
  s65_writeData(0xB0); //P7
  s65_writeData(0x02); //P8
  s65_writeData(0x00); //P9
//----------------------------
  s65_writeCmd(0xCB);// 0xCB pulse set for 64 gray scale
  for (i=0; i<29; i++)
  {
    s65_writeData(pgm_read_byte(&gcp64_0[i]));
    s65_writeData(0x00);
  }

  for (i=0; i<34; i++)
  {
    s65_writeData(pgm_read_byte(&gcp64_1[i]));
    s65_writeData(0x01);
  }

  s65_writeCmd(0xCC);//0xCC  pulse set for 16 gray scale
  for (i=0; i<15; i++)
  {
    s65_writeCmd(pgm_read_byte(&gcp16[i]));
  }

  s65_writeCmd(0xCD); //0xCD   set for gray scales
  s65_writeData(0x00);
//----------------------------

 //s65_writeCmd(0xD0);     // Oscillator select
 //s65_writeData(0x00);

  s65_writeCmd(0x94); //sleep out

  _delay_ms(7);

s65_setArea(0, 0, S65_WIDTH, S65_HEIGHT);
//  s65_writeCmd(0x15);                    //column address set 
//  s65_writeData(0x08);                //start column
///  s65_writeData(0x01);                   //start column
//  s65_writeData(0x8B);                //end column
//  s65_writeData(0x01);                   //end column                  

//  s65_writeCmd(0x75);   // page address setting
//  s65_writeData(0x00);        //page address set 
//  s65_writeData(0x8F);    //end page

//  s65_writeCmd(0xAA);  // aerea scroll setting
//  s65_writeData(0x00); //
//  s65_writeData(0xAF); //
//  s65_writeData(0xAF); //
//  s65_writeData(0x03); //

//  s65_writeCmd(0xAB); // scroll start setting
//  s65_writeData(0x00); //


  S65_RS_ENABLE(); //LOW
  s65_writeCmd(0xAF); //display on

  S65_CS_DISABLE(); //HIGH deselect display 

  return;
}

void s65_writeData(uint8_t data)
{
   
  S65_CS_ENABLE();
  s65_writeSPI(data);
  s65_writeSPI(0x00);
  S65_CS_DISABLE();

  return;
}

void s65_writeCmd(uint8_t cmd)
{
  S65_CS_ENABLE();
  S65_RS_ENABLE(); ///переводим на прием cmd

  s65_writeSPI(cmd);
  s65_writeSPI(0x00);

  S65_RS_DISABLE(); //переводим на прием data
  S65_CS_DISABLE();

  return;
}

void s65_writeSPI(uint8_t data)
{
 SPDR = data;  
 while(!(SPSR & _BV(SPIF)));

  return;
}
