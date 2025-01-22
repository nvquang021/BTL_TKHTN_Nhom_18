#ifndef __TYPES_H__
#define __TYPES_H__

#include "main.h"

typedef unsigned char   	BYTE;		// 1 byte
typedef unsigned short  	WORD;		// 2 byte
typedef signed short  	SWORD;	// 2 byte
typedef unsigned long   	DWORD;	// 4 byte
typedef signed long   	SDWORD;	// 4 byte
typedef unsigned int    	UINT;		// 2 byte
typedef unsigned char   	CHAR;		// 1 byte
typedef void            	VOID;
typedef BYTE            	BOOL;
typedef VOID 				RESULT;

typedef char 				SCHAR;

//typedef void 				VOID;
typedef BYTE* 				PBYTE;
typedef WORD* 				PWORD;
typedef DWORD* 			PDWORD;
typedef UINT* 				PUINT;
typedef CHAR* 				PCHAR;
typedef VOID* 				PVOID;

#define FALSE   			0
#define TRUE    			1
#define NULL				0

#define ON					1
#define OFF					0

#define MAX_WORD 			0xFFFF

typedef void (*SYSTEMCALLBACK)(void*);

#define INTERNAL static
#define EXTERNAL extern
#define INLINE inline

#define MASK_BIT0 0x1
#define MASK_BIT1 0x2
#define MASK_BIT2 0x4
#define MASK_BIT3 0x8
#define MASK_BIT4 0x10
#define MASK_BIT5 0x20
#define MASK_BIT6 0x40
#define MASK_BIT7 0x80

#define SETBIT(addr, bit)			((addr) |= ((DWORD)1 << (bit)))
#define CLEARBIT(addr, bit)		((addr) &= ~((DWORD)1 << (bit)))
#define CHECKBIT(addr, bit)		(((addr) & ((DWORD)1 << (bit))) == ((DWORD)1 << (bit)))

#define abs(x) ((x) > 0 ? (x) : -(x))

#define IO_OUT			0
#define IO_IN			1

#define LED_ON			1
#define LED_OFF		0

#endif /*TYPES_H_*/
