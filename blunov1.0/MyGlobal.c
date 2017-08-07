/*
 * MyGlobal.c
 *
 *  Created on: 2017��6��20��
 *      Author: Administrator
 */
#include "MyGlobal.h"

uint8 *bdAddr2Str(uint8 *buf, uint8 *pAddr )
{
  uint8       i;
  uint8        hex[] = "0123456789ABCDEF";
  uint8        *pStr = buf;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return buf;
}
