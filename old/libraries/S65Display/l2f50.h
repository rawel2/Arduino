#define l2f50_h

void s65_drawStart(void);
void s65_draw(uint16_t color);
void s65_drawStop(void);
void s65_setArea(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void s65_setCursor(uint8_t x, uint8_t y);
void s65_init(void);
void s65_writeData(uint8_t data);
void s65_writeCmd(uint8_t cmd);
void s65_writeSPI(uint8_t data);
