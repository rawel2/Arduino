#define S65Display_h

extern "C"
{
  #include <inttypes.h>
  #include <avr/pgmspace.h>
}

//Pins
# define S65_CS_PIN          (8) //PORTB 0
# define S65_RST_PIN         (9) //PORTB 1 
# define S65_RS_PIN          (10)//PORTB 2
# define S65_DAT_PIN         (11)//PORTB 3
# define S65_CLK_PIN         (13)//PORTB 5

# define S65_RST_DISABLE()   PORTB |=  (1<<1) //digitalWrite(S65_RST_PIN, HIGH)
# define S65_RST_ENABLE()    PORTB &= ~(1<<1) //digitalWrite(S65_RST_PIN, LOW)
# define S65_CS_DISABLE()    PORTB |=  (1<<0) //digitalWrite(S65_CS_PIN, HIGH)
# define S65_CS_ENABLE()     PORTB &= ~(1<<0) //digitalWrite(S65_CS_PIN, LOW)
# define S65_RS_DISABLE()    PORTB |=  (1<<2) //digitalWrite(S65_RS_PIN, HIGH)
# define S65_RS_ENABLE()     PORTB &= ~(1<<2) //digitalWrite(S65_RS_PIN, LOW)

//Display settings
#define S65_L2F50

//# define S65_WIDTH            (132)
//# define S65_HEIGHT           (176)
# define S65_WIDTH            (176)
# define S65_HEIGHT           (132)

#define RGB(r,g,b)           (((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3)) //5 red | 6 green | 5 blue

class S65Display
{
  public:
    S65Display();
    void init(uint8_t clock_div);

    void drawStart(void);
    void draw(uint16_t color);
    void drawStop(void);
    void setArea(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
    void setCursor(uint8_t x, uint8_t y);
    void clear(uint16_t color);

    void drawPixel(uint8_t x0, uint8_t y0, uint16_t color);
    void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);
    void drawRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);
    void fillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);
    void drawCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color);
    void fillCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color);

    uint8_t drawChar(uint8_t x, uint8_t y, char c, uint8_t size, uint16_t color, uint16_t bg_color);
    uint8_t drawText(uint8_t x, uint8_t y, char *s, uint8_t size, uint16_t color, uint16_t bg_color);
    uint8_t drawTextPGM(uint8_t x, uint8_t y, PGM_P s, uint8_t size, uint16_t color, uint16_t bg_color);

    uint8_t drawMLText(uint8_t x, uint8_t y, char *s, uint8_t size, uint16_t color, uint16_t bg_color);
    uint8_t drawMLTextPGM(uint8_t x, uint8_t y, PGM_P s, uint8_t size, uint16_t color, uint16_t bg_color);
};
