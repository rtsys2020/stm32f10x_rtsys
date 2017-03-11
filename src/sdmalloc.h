/*
 * sdmalloc.h
 *
 *  Created on: ? ÝÑæÑÏ?ä ???? åž.Ô.
 *      Author: Mohammad Reza Javanm
 */

#ifndef SDMALLOC_H_
#define SDMALLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

void *sdpvPortMalloc( size_t xWantedSize );
void sdvPortFree( void *pv );
size_t sdxPortGetFreeHeapSize( void );
size_t sdxPortGetMinimumEverFreeHeapSize( void );
void sdvPortInitialiseBlocks( void );

#ifdef __cplusplus
} //extern "C"
#endif

#endif /* SDMALLOC_H_ */
