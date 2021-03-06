/*
  arduino I2C to Colorduino demo
   
  based on 
  -arduino firmware by michael vogt <michu@neophob.com>
  -blinkm firmware by thingM
  -"daft punk" firmware by Scott C / ThreeFN 
   
  libraries to patch:
  Wire: 
   	utility/twi.h: #define TWI_FREQ 400000L (was 100000L)
                       #define TWI_BUFFER_LENGTH 70 (was 32)
   	wire.h:        #define BUFFER_LENGTH 70 (was 32)
 
 
 */

#include <Wire.h>



#define START_OF_DATA 0x10       //data markers
#define END_OF_DATA 0x20         //data markers
#define DEST_I2C_ADDR 5          //set destination I2C address (must match firmware in Colorduino module)

#define SCREENSIZEX 8            //num of LEDs accross
#define SCREENSIZEY 8            //num of LEDs down
#define DS3231_ADDR 0x68

byte display_byte[3][4][64];        //display array - 64 bytes x 3 colours 

extern unsigned char font8_8[92][8];

typedef union {
	uint8_t bytes[7];
	struct {
		uint8_t ss;
		uint8_t mm;
		uint8_t hh;
		uint8_t dayofwek;
		uint8_t day;
		uint8_t month;
		uint8_t year;
	};
} TDATETIME;


typedef struct {
	int8_t cel;
	uint8_t fract;
} TTEMP;


//setup for plasma
typedef struct
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
} ColorRGB;

//a color with 3 components: h, s and v
typedef struct 
{
  unsigned char h;
  unsigned char s;
  unsigned char v;
} ColorHSV;

void DS3231_get_datetime( TDATETIME * dt );
void DS3231_get_temp( TTEMP * tmp );

long paletteShift;
TDATETIME datetime;
TTEMP temperature;

 
void setup() 
{
  Wire.begin(); // join i2c bus (address optional for master)  
  // Serial.begin(9600);
  paletteShift = 0;
  DS3231_init();
  //DS3231_set_time(13,1,0);
  DS3231_get_datetime(&datetime );
  DS3231_get_temp(&temperature);
}

void loop()
{
   plasma_morph();
}


void plasma_morph()
{
  unsigned char x,y;
  float value;
  ColorRGB colorRGB;
  ColorHSV colorHSV;
  unsigned char x0; 
  int y0; 
  unsigned char neg;

  for(x = 0; x <SCREENSIZEX; x++) {
    for(y = 0; y < SCREENSIZEY; y++)
      {
	value = sin(dist(x + paletteShift, y, 128.0, 128.0) / 8.0)
	  + sin(dist(x, y, 64.0, 64.0) / 8.0)
	  + sin(dist(x, y + paletteShift / 7, 192.0, 64) / 7.0)
	  + sin(dist(x, y, 192.0, 100.0) / 8.0);
	colorHSV.h=(unsigned char)((value) * 128)&0xff;
	colorHSV.s=255; 
	colorHSV.v=100;
	HSVtoRGB(&colorRGB, &colorHSV);
	
	display(x, y, colorRGB.r, colorRGB.g, colorRGB.b);
      }
  }
  paletteShift++;
  
  x0 = 0;
//  y0 = paletteShift%300 - 8;
//  if (y0 > 0) {y0=0;}
  
  y0 = paletteShift%300;
  
  if (y0 < 9)
    {y0 = y0-8;}
  else if (y0 <241)
    {y0 = 0;}
  else if (y0 <250)  
    {y0 = y0 - 241;}
  else if (y0 <259)  
    {y0 = y0 - 258;}
  else if (y0 <291)  
    {y0 = 0;}
  else
    {y0 = y0 - 291;}  
    
  neg = paletteShift / 600 %2;
  
  
  if(paletteShift % 300 < 250)
  {
    if (paletteShift % 30 == 0)
    {DS3231_get_datetime(&datetime );};
  
    DispShowChar((datetime.hh/10)+48,  x0,  x0,  x0, y0, 3, neg, 0);
    DispShowChar((datetime.hh%10)+48,  x0,  x0,  x0, y0, 2, neg, 0);
    DispShowChar((datetime.mm/10)+48,  x0,  x0,  x0, y0, 1, neg, paletteShift%12 > 2);
    DispShowChar((datetime.mm%10)+48,  x0,  x0,  x0, y0, 0, neg, 0);
  }
  else
  {
    if (paletteShift % 300 == 250)
    {DS3231_get_temp(&temperature);};
  
    DispShowChar((temperature.cel/10)+48,  x0,  x0,  x0, y0, 3, neg, 0);
    DispShowChar((temperature.cel%10)+48,  x0,  x0,  x0, y0, 2, neg, 0);
    DispShowChar('"',  x0,  x0,  x0, y0, 1, neg, 0);
    DispShowChar('C',  x0,  x0,  x0, y0, 0, neg, 0);
    
  }
   
  //DispShowChar('K',  x0,  x0,  x0, y0, 3, paletteShift>1000);
  //DispShowChar('u',  x0,  x0,  x0, y0, 2, paletteShift>1000);
  //DispShowChar('b',  x0,  x0,  x0, y0, 1, paletteShift>1000);
  //DispShowChar('a',  x0,  x0,  x0, y0, 0, paletteShift>1000);

  update_display(DEST_I2C_ADDR,0);
  update_display(DEST_I2C_ADDR,1);
  update_display(DEST_I2C_ADDR,2);
  update_display(DEST_I2C_ADDR,3);
 
}


//update display buffer using x,y,r,g,b format
void display(byte x, byte y, byte r, byte g, byte b) {
  byte p1 = (y*8)+x;   //convert from x,y to pixel number in array
  byte p2 = ((y)*8)+(7-x);
  byte p3 = ((7-y)*8)+x;
  byte p4 = ((7-y)*8)+(7-x);
  
  display_byte[0][0][p1] = r;
  display_byte[1][0][p1] = g;
  display_byte[2][0][p1] = b;

  display_byte[0][1][p2] = r;
  display_byte[1][1][p2] = g;
  display_byte[2][1][p2] = b;
  
  display_byte[0][2][p1] = r;
  display_byte[1][2][p1] = g;
  display_byte[2][2][p1] = b;

  display_byte[0][3][p2] = r;
  display_byte[1][3][p2] = g;
  display_byte[2][3][p2] = b;
  
  /*
  display_byte[0][2][p3] = r;
  display_byte[1][2][p3] = g;
  display_byte[2][2][p3] = b;

  display_byte[0][3][p4] = r;
  display_byte[1][3][p4] = g;
  display_byte[2][3][p4] = b;
  */
  
}


//send display buffer to display 
void update_display(byte addr, byte addr2) {   
  BlinkM_sendBuffer(addr+addr2, 0, display_byte[0][addr2]);
  BlinkM_sendBuffer(addr+addr2, 1, display_byte[1][addr2]);   
  BlinkM_sendBuffer(addr+addr2, 2, display_byte[2][addr2]);  
}


//send data via I2C to a client
static byte BlinkM_sendBuffer(byte addr, byte col, byte* disp_data) {
  Wire.beginTransmission(addr);
  Wire.write(START_OF_DATA);
  Wire.write(col);
  Wire.write(disp_data, 64);
  Wire.write(END_OF_DATA);
  return Wire.endTransmission();
}


//plasma convert
//Converts an HSV color to RGB color
void HSVtoRGB(void *vRGB, void *vHSV) 
{
  float r, g, b, h, s, v; //this function works with floats between 0 and 1
  float f, p, q, t;
  int i;
  ColorRGB *colorRGB=(ColorRGB *)vRGB;
  ColorHSV *colorHSV=(ColorHSV *)vHSV;

  h = (float)(colorHSV->h / 256.0);
  s = (float)(colorHSV->s / 256.0);
  v = (float)(colorHSV->v / 256.0);

  //if saturation is 0, the color is a shade of grey
  if(s == 0.0) {
    b = v;
    g = b;
    r = g;
  }
  //if saturation > 0, more complex calculations are needed
  else
  {
    h *= 6.0; //to bring hue to a number between 0 and 6, better for the calculations
    i = (int)(floor(h)); //e.g. 2.7 becomes 2 and 3.01 becomes 3 or 4.9999 becomes 4
    f = h - i;//the fractional part of h

    p = (float)(v * (1.0 - s));
    q = (float)(v * (1.0 - (s * f)));
    t = (float)(v * (1.0 - (s * (1.0 - f))));

    switch(i)
    {
      case 0: r=v; g=t; b=p; break;
      case 1: r=q; g=v; b=p; break;
      case 2: r=p; g=v; b=t; break;
      case 3: r=p; g=q; b=v; break;
      case 4: r=t; g=p; b=v; break;
      case 5: r=v; g=p; b=q; break;
      default: r = g = b = 0; break;
    }
  }
  colorRGB->r = (int)(r * 255.0);
  colorRGB->g = (int)(g * 255.0);
  colorRGB->b = (int)(b * 255.0);
}

 
void DispShowChar(char chr,unsigned char R,unsigned char G,unsigned char B,char bias,char disp, char neg, char dwukropek)
{
  unsigned char i,j,temp;
  unsigned char Char;
  unsigned char chrtemp[24] = {0};
  unsigned char p1;
  
  if ((bias > 8) || (bias < -8))
    return;

  Char = chr - 32;
  
  j = 8 - bias;
  
  for(i = 0;i< 8;i++){
    chrtemp[j] = pgm_read_byte(&(font8_8[Char][i]));    
    j++;
  }  
  
  for(i = 0;i < 8;i++)
  {
    if((dwukropek == 1) and (i==0))
    {temp = B00100100;}
    else
    {temp = chrtemp[i+8];}
    
    for(j = 0;j < 8;j++)
    {
      p1 = (j*8)+i;
      if((temp & 0x80) and (neg))
      {
        display_byte[0][disp][p1] = R;
        display_byte[1][disp][p1] = G;
        display_byte[2][disp][p1] = B;
      }
      
      if(!(temp & 0x80) and (!neg))
      {
        display_byte[0][disp][p1] = R;
        display_byte[1][disp][p1] = G;
        display_byte[2][disp][p1] = B;
      }
      temp = temp << 1;
    }
  }

}


float dist(float a, float b, float c, float d) 
{
  return sqrt((c-a)*(c-a)+(d-b)*(d-b));
}



void DS3231_init( void ) {
	uint8_t ctrl = 0;
	TWI_write_buf( DS3231_ADDR, 0x0e, 1, &ctrl );
}

void DS3231_get_datetime( TDATETIME * dt ) {
	uint8_t i;
	uint8_t buf[7];
	TWI_read_buf( DS3231_ADDR, 0x00, 7, buf );
	for( i=0; i<7; i++ ) dt->bytes[i] = bcd2dec( buf[i] );
}

void DS3231_set_time( uint8_t hh, uint8_t mm, uint8_t ss ) {
	uint8_t buf[3];
	buf[0]=dec2bcd(ss);
	buf[1]=dec2bcd(mm);
	buf[2]=dec2bcd(hh);
	TWI_write_buf( DS3231_ADDR, 0x00, 3, buf );
}

void DS3231_set_date( uint8_t year, uint8_t month, uint8_t day, uint8_t dayofweek ) {
	uint8_t buf[4];
	buf[0]=dayofweek;
	buf[1]=dec2bcd(day);
	buf[2]=dec2bcd(month);
	buf[3]=dec2bcd(year);
	TWI_write_buf( DS3231_ADDR, 0x03, 4, buf );
}


void DS3231_get_temp( TTEMP * tmp ) {
	uint8_t buf[2];
	TWI_read_buf( DS3231_ADDR, 0x11, 2, buf );
	tmp->cel = buf[0] ;
	tmp->fract = buf[1]>>6;

	TWI_read_buf( DS3231_ADDR, 0x0e, 1, buf );
	buf[0] |= (1<<5);
	TWI_write_buf( DS3231_ADDR, 0x0e, 1, buf );
}

// konwersja liczby dziesiętnej na BCD
uint8_t dec2bcd(uint8_t dec) {
return ((dec / 10)<<4) | (dec % 10);
}

// konwersja liczby BCD na dziesięną
uint8_t bcd2dec(uint8_t bcd) {
    return ((((bcd) >> 4) & 0x0F) * 10) + ((bcd) & 0x0F);
}


void TWI_write_buf( uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf){
  Wire.beginTransmission(SLA);
  Wire.write(adr);
  Wire.write(buf,len);
  Wire.endTransmission();
 
};

void TWI_read_buf(uint8_t SLA, uint8_t adr, uint8_t len, uint8_t *buf){
    Wire.beginTransmission(SLA);
    Wire.write(adr);
    Wire.endTransmission();
    Wire.requestFrom(SLA, len);
    for(int i=0; i<len; i++)
      {
        buf[i]=Wire.read();
      }

};






