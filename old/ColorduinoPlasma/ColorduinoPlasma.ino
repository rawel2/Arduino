#include <Colorduino.h>

typedef struct
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
} 
ColorRGB;

//a color with 3 components: h, s and v
typedef struct 
{
  unsigned char h;
  unsigned char s;
  unsigned char v;
} 
ColorHSV;

long paletteShift;

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
  if (s == 0.0)     {    b = v;    g = b;    r = g;    }
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
          case 0: 
            r=v;       g=t;       b=p; 
            break;
          case 1: 
            r=q;      g=v;       b=p; 
            break;
          case 2: 
            r=p;       g=v;       b=t; 
            break;
          case 3: 
            r=p;       g=q;       b=v; 
            break;
          case 4: 
            r=t;       g=p;       b=v; 
            break;
          case 5: 
            r=v;       g=p;       b=q; 
            break;
          default: 
            r = g = b = 0; 
            break;
        }
    }
    colorRGB->r = (int)(r * 255.0);
    colorRGB->g = (int)(g * 255.0);
    colorRGB->b = (int)(b * 255.0);
}

float dist(float a, float b, float c, float d) 
{
  return sqrt((c-a)*(c-a)+(d-b)*(d-b));
}


void plasma_morph()
{
  unsigned char x,y;
  float value;
  ColorRGB colorRGB;
  ColorHSV colorHSV;

  for(y = 0; y < ColorduinoScreenHeight; y++)
    for(x = 0; x < ColorduinoScreenWidth; x++) {
      {
        value = sin(dist(x + paletteShift, y, 128.0, 128.0) / 8.0)
          + sin(dist(x, y, 64.0, 64.0) / 8.0)
            + sin(dist(x, y + paletteShift / 7, 192.0, 64) / 7.0)
              + sin(dist(x, y, 192.0, 100.0) / 8.0);
        colorHSV.h=(unsigned char)((value) * 128)&0xff;
        colorHSV.s=255; // 0 - biały nasycenie 
        colorHSV.v=50; // jaskrawosc 
        HSVtoRGB(&colorRGB, &colorHSV);

        Colorduino.SetPixel(x, y, colorRGB.r, colorRGB.g, colorRGB.b);
      }
    }
  paletteShift++;

  Colorduino.FlipPage(); // swap screen buffers to show it
}

void Litera_noFlip(unsigned char lit,unsigned char R,unsigned char G,unsigned char B)
{//na razie litera nas nie obchodzi

Colorduino.SetPixel(1, 1, R, G, B);
Colorduino.SetPixel(1, 2, R, G, B);
Colorduino.SetPixel(1, 3, R, G, B);
Colorduino.SetPixel(1, 4, R, G, B);
Colorduino.SetPixel(1, 5, R, G, B);
Colorduino.SetPixel(1, 6, R, G, B);
Colorduino.SetPixel(2, 1, R, G, B);
Colorduino.SetPixel(2, 2, R, G, B);
Colorduino.SetPixel(2, 3, R, G, B);
Colorduino.SetPixel(2, 4, R, G, B);
Colorduino.SetPixel(2, 5, R, G, B);
Colorduino.SetPixel(2, 6, R, G, B);
Colorduino.SetPixel(3, 1, R, G, B);
Colorduino.SetPixel(3, 4, R, G, B);
Colorduino.SetPixel(3, 6, R, G, B);
Colorduino.SetPixel(4, 1, R, G, B);
Colorduino.SetPixel(4, 4, R, G, B);
Colorduino.SetPixel(4, 6, R, G, B);
Colorduino.SetPixel(5, 1, R, G, B);
Colorduino.SetPixel(5, 4, R, G, B);
Colorduino.SetPixel(5, 6, R, G, B);
Colorduino.SetPixel(6, 2, R, G, B);
Colorduino.SetPixel(6, 3, R, G, B);
Colorduino.SetPixel(6, 5, R, G, B);

};

void plasma_morph2()
{
  unsigned char x,y;
  float value;
  ColorRGB colorRGB;
  ColorHSV colorHSV;

  for(y = 0; y <= 255; y++)
  {
        colorHSV.h=(unsigned char)(y)&0xff;
        colorHSV.s=255; // 0 - biały nasycenie 
        colorHSV.v=20; // jaskrawosc 
        HSVtoRGB(&colorRGB, &colorHSV);
        Colorduino.ColorFill_noFlip(colorRGB.r, colorRGB.g, colorRGB.b);
        
        colorHSV.h=(unsigned char)(y+128)&0xff;
        colorHSV.s=255; // 0 - biały nasycenie 
        colorHSV.v=200; // jaskrawosc 
        HSVtoRGB(&colorRGB, &colorHSV);
        Litera_noFlip('B',colorRGB.r, colorRGB.g, colorRGB.b);
        //Colorduino.RectangleFill_noFlip(2, 2,5, 5,colorRGB.r, colorRGB.g, colorRGB.b);
        delay(50);
        Colorduino.FlipPage();
        //Colorduino.SetPixel(x, y, colorRGB.r, colorRGB.g, colorRGB.b);
      }
     
}

void setup()
{
  Colorduino.Init(); // initialize the board

  // compensate for relative intensity differences in R/G/B brightness
  // array of 6-bit base values for RGB (0~63)
  // whiteBalVal[0]=red
  // whiteBalVal[1]=green
  // whiteBalVal[2]=blue
  unsigned char whiteBalVal[3] = {30,63,57  }; // for LEDSEE 6x6cm round matrix

  Colorduino.SetWhiteBal(whiteBalVal);

  // start with morphing plasma, but allow going to color cycling if desired.
  paletteShift=128000;

  Colorduino.ColorFill(255,255,255);
  delay(3000);
}

void loop()
{
  plasma_morph2();
}

