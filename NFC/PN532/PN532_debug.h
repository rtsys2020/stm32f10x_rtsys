#ifndef __DEBUG_H__
#define __DEBUG_H__

//#define DEBUG

#include "Arduino.h"

#define DEBUG
#ifdef DEBUG
#define DMSG(args...)       Serial1.print(args)
#define DMSG_STR(str)       Serial1.println(str)
#define DMSG_HEX(num)       Serial1.print(' '); Serial1.print(num, HEX)
#define DMSG_INT(num)       Serial1.print(' '); Serial1.print(num)
#else
#define DMSG(args...)
#define DMSG_STR(str)
#define DMSG_HEX(num)
#define DMSG_INT(num)
#endif

#endif
