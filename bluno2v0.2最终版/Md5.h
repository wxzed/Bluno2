/*
 * Md5.h
 *
 *  Created on: 2017��5��15��
 *      Author: Administrator
 */

#ifndef MD5_H_
#define MD5_H_

typedef unsigned long UINT4;
typedef unsigned char *POINTER;

/* MD5 context. */
typedef struct {
  /* state (ABCD) */
  /*�ĸ�32bits�������ڴ�����ռ���õ�����ϢժҪ������Ϣ���ȡ�512bitsʱ��Ҳ���ڴ��ÿ��512bits���м���*/
  UINT4 state[4];

  /* number of bits, modulo 2^64 (lsb first) */
  /*�洢ԭʼ��Ϣ��bits������,����������bits���Ϊ 2^64 bits����Ϊ2^64��һ��64λ�������ֵ*/
  UINT4 count[2];

  /* input buffer */
  /*����������Ϣ�Ļ�������512bits*/
  unsigned char buffer[64];
} MD5_CTX;

void MD5Init(MD5_CTX *);
void MD5Update(MD5_CTX *, const unsigned char *, unsigned int);
void MD5Final(unsigned char [16], MD5_CTX *);
#endif /* MD5_H_ */
