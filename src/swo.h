/*
 * swo.h
 *
 *  Created on: 10.10.2016
 *      Author: Erich Styger Local
 */

#ifndef SOURCES_SWO_H_
#define SOURCES_SWO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void SWO_PrintChar(char c, uint8_t portNumber);
void SWO_PrintString(const char *s, uint8_t portNumber);

void SWO_Init(uint32_t portBits, uint32_t cpuCoreFreqHz);

#ifdef __cplusplus
} //extern "C"
#endif

#endif /* SOURCES_SWO_H_ */
