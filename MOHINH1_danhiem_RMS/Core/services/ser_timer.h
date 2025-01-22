#ifndef __SER_TIMER_H__
#define __SER_TIMER_H__

#include "datatypes.h"
#include "main.h"

#define MAXCALLBACKFUNC					26
//#define ALL                        MAXCALLBACKFUNC

typedef struct
{
	SYSTEMCALLBACK 	CallbackFunc;
	WORD			wTickCount;
	BYTE 			nSecondCount;
	BYTE			nState;
	PVOID			pData;
} CallbackFuncTypeDef, *PCallbackFuncTypeDef;

extern CallbackFuncTypeDef arCallbackFunc[MAXCALLBACKFUNC];

VOID CancelTimer(SYSTEMCALLBACK Callback, PVOID pData);
VOID StartShortTimer(WORD wTickCount, SYSTEMCALLBACK Callback, PVOID pData);
VOID StartLongTimer(WORD wSecondCount, SYSTEMCALLBACK Callback, PVOID pData);
BOOL IsRunning(SYSTEMCALLBACK Callback, PVOID pData);
VOID ProcessTimerEvents(VOID);

#endif // __TIMER_H__
