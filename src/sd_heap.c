/*
 * sd_heap.c
 *
 *  Created on: ? ÝÑæÑÏ?ä ???? åž.Ô.
 *      Author: Mohammad Reza Javanmardi
 */

/*
 FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
 All rights reserved

 VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

 ***************************************************************************
 >>!   NOTE: The modification to the GPL is included to allow you to     !<<
 >>!   distribute a combined work that includes FreeRTOS without being   !<<
 >>!   obliged to provide the source code for proprietary components     !<<
 >>!   outside of the FreeRTOS kernel.                                   !<<
 ***************************************************************************

 FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  Full license text is available on the following
 link: http://www.freertos.org/a00114.html

 ***************************************************************************
 *                                                                       *
 *    FreeRTOS provides completely free yet professionally developed,    *
 *    robust, strictly quality controlled, supported, and cross          *
 *    platform software that is more than just the market leader, it     *
 *    is the industry's de facto standard.                               *
 *                                                                       *
 *    Help yourself get started quickly while simultaneously helping     *
 *    to support the FreeRTOS project by purchasing a FreeRTOS           *
 *    tutorial book, reference manual, or both:                          *
 *    http://www.FreeRTOS.org/Documentation                              *
 *                                                                       *
 ***************************************************************************

 http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
 the FAQ page "My application does not run, what could be wrong?".  Have you
 defined configASSERT()?

 http://www.FreeRTOS.org/support - In return for receiving this top quality
 embedded software for free we request you assist our global community by
 participating in the support forum.

 http://www.FreeRTOS.org/training - Investing in training allows your team to
 be as productive as possible as early as possible.  Now you can receive
 FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
 Ltd, and the world's leading authority on the world's leading RTOS.

 http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
 including FreeRTOS+Trace - an indispensable productivity tool, a DOS
 compatible FAT file system, and our tiny thread aware UDP/IP stack.

 http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
 Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

 http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
 Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
 licenses offer ticketed support, indemnification and commercial middleware.

 http://www.SafeRTOS.com - High Integrity Systems also provide a safety
 engineered and independently SIL3 certified version for use in safety and
 mission critical applications that require provable dependability.

 1 tab == 4 spaces!
 */

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */

#include "sdmalloc.h"

/* Block sizes must not get too small. */
#define sdheapMINIMUM_BLOCK_SIZE	( ( size_t ) ( sdxHeapStructSize * 2 ) )

/* Assumes 8bit bytes! */
#define sdheapBITS_PER_BYTE		( ( size_t ) 8 )

#define sdconfigTOTAL_HEAP_SIZE 1024 * 1024 * 32

/* Allocate the memory for the heap. */
static uint8_t *sducHeap = (uint8_t *) 0xc0000000; //[ configTOTAL_HEAP_SIZE ];

/* Define the linked list structure.  This is used to link free blocks in order
 of their memory address. */
typedef struct sdA_BLOCK_LINK {
	struct sdA_BLOCK_LINK *pxNextFreeBlock; /*<< The next free block in the list. */
	size_t xBlockSize; /*<< The size of the free block. */
} sdBlockLink_t;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void sdprvInsertBlockIntoFreeList(sdBlockLink_t *pxBlockToInsert);

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void sdprvHeapInit(void);

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
 block must by correctly byte aligned. */
static const size_t sdxHeapStructSize = ((sizeof(sdBlockLink_t)
		+ (((size_t) portBYTE_ALIGNMENT_MASK) - (size_t) 1))
		& ~((size_t) portBYTE_ALIGNMENT_MASK));

/* Create a couple of list links to mark the start and end of the list. */
static sdBlockLink_t sdxStart, *sdpxEnd = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
 fragmentation. */
static size_t sdxFreeBytesRemaining = 0U;
static size_t sdxMinimumEverFreeBytesRemaining = 0U;

/* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
 member of an BlockLink_t structure is set then the block belongs to the
 application.  When the bit is free the block is still part of the free heap
 space. */
static size_t sdxBlockAllocatedBit = 0;

/*-----------------------------------------------------------*/

void *sdpvPortMalloc(size_t xWantedSize) {
	sdBlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
	void *pvReturn = NULL;

	vTaskSuspendAll();
	{
		/* If this is the first call to malloc then the heap will require
		 Initialization to setup the list of free blocks. */
		if (sdpxEnd == NULL) {
			sdprvHeapInit();
		} else {
			mtCOVERAGE_TEST_MARKER();
		}

		/* Check the requested block size is not so large that the top bit is
		 set.  The top bit of the block size member of the BlockLink_t structure
		 is used to determine who owns the block - the application or the
		 kernel, so it must be free. */
		if ((xWantedSize & sdxBlockAllocatedBit) == 0) {
			/* The wanted size is increased so it can contain a BlockLink_t
			 structure in addition to the requested amount of bytes. */
			if (xWantedSize > 0) {
				xWantedSize += sdxHeapStructSize;

				/* Ensure that blocks are always aligned to the required number
				 of bytes. */
				if ((xWantedSize & portBYTE_ALIGNMENT_MASK) != 0x00) {
					/* Byte alignment required. */
					xWantedSize += ( portBYTE_ALIGNMENT
							- (xWantedSize & portBYTE_ALIGNMENT_MASK));
					configASSERT(( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0);
				} else {
					mtCOVERAGE_TEST_MARKER();
				}
			} else {
				mtCOVERAGE_TEST_MARKER();
			}

			if ((xWantedSize > 0) && (xWantedSize <= sdxFreeBytesRemaining)) {
				/* Traverse the list from the start	(lowest address) block until
				 one	of adequate size is found. */
				pxPreviousBlock = &sdxStart;
				pxBlock = sdxStart.pxNextFreeBlock;
				while ((pxBlock->xBlockSize < xWantedSize)
						&& (pxBlock->pxNextFreeBlock != NULL)) {
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				/* If the end marker was reached then a block of adequate size
				 was	not found. */
				if (pxBlock != sdpxEnd) {
					/* Return the memory space pointed to - jumping over the
					 BlockLink_t structure at its start. */
					pvReturn =
							(void *) (((uint8_t *) pxPreviousBlock->pxNextFreeBlock)
									+ sdxHeapStructSize);

					/* This block is being returned for use so must be taken out
					 of the list of free blocks. */
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					/* If the block is larger than required it can be split into
					 two. */
					if ((pxBlock->xBlockSize - xWantedSize)
							> sdheapMINIMUM_BLOCK_SIZE) {
						/* This block is to be split into two.  Create a new
						 block following the number of bytes requested. The void
						 cast is used to prevent byte alignment warnings from the
						 compiler. */
						pxNewBlockLink = (void *) (((uint8_t *) pxBlock)
								+ xWantedSize);
						configASSERT(
								( ( ( uint32_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0);

						/* Calculate the sizes of two blocks split from the
						 single block. */
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize
								- xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						/* Insert the new block into the list of free blocks. */
						sdprvInsertBlockIntoFreeList((pxNewBlockLink));
					} else {
						mtCOVERAGE_TEST_MARKER();
					}

					sdxFreeBytesRemaining -= pxBlock->xBlockSize;

					if (sdxFreeBytesRemaining
							< sdxMinimumEverFreeBytesRemaining) {
						sdxMinimumEverFreeBytesRemaining =
								sdxFreeBytesRemaining;
					} else {
						mtCOVERAGE_TEST_MARKER();
					}

					/* The block is being returned - it is allocated and owned
					 by the application and has no "next" block. */
					pxBlock->xBlockSize |= sdxBlockAllocatedBit;
					pxBlock->pxNextFreeBlock = NULL;
				} else {
					mtCOVERAGE_TEST_MARKER();
				}
			} else {
				mtCOVERAGE_TEST_MARKER();
			}
		} else {
			mtCOVERAGE_TEST_MARKER();
		}

		traceMALLOC( pvReturn, xWantedSize );
	}
	(void) xTaskResumeAll();

#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if (pvReturn == NULL) {
			extern void vApplicationMallocFailedHook(void);
			vApplicationMallocFailedHook();
		} else {
			mtCOVERAGE_TEST_MARKER();
		}
	}
#endif

	configASSERT(( ( ( uint32_t ) pvReturn ) & portBYTE_ALIGNMENT_MASK ) == 0);
	return pvReturn;
}
/*-----------------------------------------------------------*/

void sdvPortFree(void *pv) {
	uint8_t *puc = (uint8_t *) pv;
	sdBlockLink_t *pxLink;

	if (pv != NULL) {
		/* The memory being freed will have an BlockLink_t structure immediately
		 before it. */
		puc -= sdxHeapStructSize;

		/* This casting is to keep the compiler from issuing warnings. */
		pxLink = (void *) puc;

		/* Check the block is actually allocated. */
		configASSERT((pxLink->xBlockSize & sdxBlockAllocatedBit) != 0);
		configASSERT(pxLink->pxNextFreeBlock == NULL);

		if ((pxLink->xBlockSize & sdxBlockAllocatedBit) != 0) {
			if (pxLink->pxNextFreeBlock == NULL) {
				/* The block is being returned to the heap - it is no longer
				 allocated. */
				pxLink->xBlockSize &= ~sdxBlockAllocatedBit;

				vTaskSuspendAll();
				{
					/* Add this block to the list of free blocks. */
					sdxFreeBytesRemaining += pxLink->xBlockSize;
					traceFREE( pv, pxLink->xBlockSize );
					sdprvInsertBlockIntoFreeList(((sdBlockLink_t *) pxLink));
				}
				(void) xTaskResumeAll();
			} else {
				mtCOVERAGE_TEST_MARKER();
			}
		} else {
			mtCOVERAGE_TEST_MARKER();
		}
	}
}
/*-----------------------------------------------------------*/

size_t sdxPortGetFreeHeapSize(void) {
	return sdxFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t sdxPortGetMinimumEverFreeHeapSize(void) {
	return sdxMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void sdvPortInitialiseBlocks(void) {
	/* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

static void sdprvHeapInit(void) {
	sdBlockLink_t *pxFirstFreeBlock;
	uint8_t *pucAlignedHeap;
	uint32_t ulAddress;
	size_t xTotalHeapSize = sdconfigTOTAL_HEAP_SIZE;

	/* Ensure the heap starts on a correctly aligned boundary. */
	ulAddress = (uint32_t) sducHeap;

	if ((ulAddress & portBYTE_ALIGNMENT_MASK) != 0) {
		ulAddress += ( portBYTE_ALIGNMENT - 1);
		ulAddress &= ~((uint32_t) portBYTE_ALIGNMENT_MASK);
		xTotalHeapSize -= ulAddress - (uint32_t) sducHeap;
	}

	pucAlignedHeap = (uint8_t *) ulAddress;

	/* xStart is used to hold a pointer to the first item in the list of free
	 blocks.  The void cast is used to prevent compiler warnings. */
	sdxStart.pxNextFreeBlock = (void *) pucAlignedHeap;
	sdxStart.xBlockSize = (size_t) 0;

	/* pxEnd is used to mark the end of the list of free blocks and is inserted
	 at the end of the heap space. */
	ulAddress = ((uint32_t) pucAlignedHeap) + xTotalHeapSize;
	ulAddress -= sdxHeapStructSize;
	ulAddress &= ~((uint32_t) portBYTE_ALIGNMENT_MASK);
	sdpxEnd = (void *) ulAddress;
	sdpxEnd->xBlockSize = 0;
	sdpxEnd->pxNextFreeBlock = NULL;

	/* To start with there is a single free block that is sized to take up the
	 entire heap space, minus the space taken by pxEnd. */
	pxFirstFreeBlock = (void *) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = ulAddress - (uint32_t) pxFirstFreeBlock;
	pxFirstFreeBlock->pxNextFreeBlock = sdpxEnd;

	/* Only one block exists - and it covers the entire usable heap space. */
	sdxMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
	sdxFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

	/* Work out the position of the top bit in a size_t variable. */
	sdxBlockAllocatedBit = ((size_t) 1)
			<< ((sizeof(size_t) * sdheapBITS_PER_BYTE) - 1);
}
/*-----------------------------------------------------------*/

static void sdprvInsertBlockIntoFreeList(sdBlockLink_t *pxBlockToInsert) {
	sdBlockLink_t *pxIterator;
	uint8_t *puc;

	/* Iterate through the list until a block is found that has a higher address
	 than the block being inserted. */
	for (pxIterator = &sdxStart; pxIterator->pxNextFreeBlock < pxBlockToInsert;
			pxIterator = pxIterator->pxNextFreeBlock) {
		/* Nothing to do here, just iterate to the right position. */
	}

	/* Do the block being inserted, and the block it is being inserted after
	 make a contiguous block of memory? */
	puc = (uint8_t *) pxIterator;
	if ((puc + pxIterator->xBlockSize) == (uint8_t *) pxBlockToInsert) {
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	} else {
		mtCOVERAGE_TEST_MARKER();
	}

	/* Do the block being inserted, and the block it is being inserted before
	 make a contiguous block of memory? */
	puc = (uint8_t *) pxBlockToInsert;
	if ((puc + pxBlockToInsert->xBlockSize)
			== (uint8_t *) pxIterator->pxNextFreeBlock) {
		if (pxIterator->pxNextFreeBlock != sdpxEnd) {
			/* Form one big block from the two blocks. */
			pxBlockToInsert->xBlockSize +=
					pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock =
					pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		} else {
			pxBlockToInsert->pxNextFreeBlock = sdpxEnd;
		}
	} else {
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	/* If the block being inserted plugged a gab, so was merged with the block
	 before and the block after, then it's pxNextFreeBlock pointer will have
	 already been set, and should not be set here as that would make it point
	 to itself. */
	if (pxIterator != pxBlockToInsert) {
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	} else {
		mtCOVERAGE_TEST_MARKER();
	}
}

