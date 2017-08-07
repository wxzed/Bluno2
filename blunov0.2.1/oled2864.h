/*
 * oled2864.h
 *
 *  Created on: 2017Äê4ÔÂ7ÈÕ
 *      Author: Administrator
 */

#ifndef OLED2864_H_
#define OLED2864_H_
#include "stdio.h"


void oled_begin();
void oled_init();
void show(uint8_t x, uint8_t y, uint8_t DATA);
void fill(unsigned char dat);
//void fill(uint8_t x,uint8_t y,unsigned char dat);
void setCursor(uint8_t x, uint8_t y);
void SSD1306();
void Reset();
//void oled_print(uint8_t x, uint8_t y,float f);
//void oled_print(uint8_t x, uint8_t y,int n);
void oled_print_int(uint8_t x, uint8_t y,int n);
void oled_print(uint8_t x, uint8_t y,const char *str);

#endif /* OLED2864_H_ */
