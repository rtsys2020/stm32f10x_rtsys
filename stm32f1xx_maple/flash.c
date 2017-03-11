/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/flash.c
 * @brief Flash management functions
 */

#include <libmaple/libmaple_types.h>
#include <libmaple/flash.h>


#define wTransferSize               0x0400   /* wTransferSize   = 1024 bytes */

/* Flash Access Control Register bits */
#define ACR_LATENCY_Mask         ((uint32_t)0x00000038)
#define ACR_HLFCYA_Mask          ((uint32_t)0xFFFFFFF7)
#define ACR_PRFTBE_Mask          ((uint32_t)0xFFFFFFEF)

/* Flash Access Control Register bits */
#define ACR_PRFTBS_Mask          ((uint32_t)0x00000020)

/* Flash Control Register bits */
#define CR_PG_Set                ((uint32_t)0x00000001)
#define CR_PG_Reset              ((uint32_t)0x00001FFE)
#define CR_PER_Set               ((uint32_t)0x00000002)
#define CR_PER_Reset             ((uint32_t)0x00001FFD)
#define CR_MER_Set               ((uint32_t)0x00000004)
#define CR_MER_Reset             ((uint32_t)0x00001FFB)
#define CR_OPTPG_Set             ((uint32_t)0x00000010)
#define CR_OPTPG_Reset           ((uint32_t)0x00001FEF)
#define CR_OPTER_Set             ((uint32_t)0x00000020)
#define CR_OPTER_Reset           ((uint32_t)0x00001FDF)
#define CR_STRT_Set              ((uint32_t)0x00000040)
#define CR_LOCK_Set              ((uint32_t)0x00000080)

/* FLASH Mask */
#define RDPRT_Mask               ((uint32_t)0x00000002)
#define WRP0_Mask                ((uint32_t)0x000000FF)
#define WRP1_Mask                ((uint32_t)0x0000FF00)
#define WRP2_Mask                ((uint32_t)0x00FF0000)
#define WRP3_Mask                ((uint32_t)0xFF000000)
#define OB_USER_BFB2             ((uint16_t)0x0008)

/* FLASH Keys */
#define RDP_Key                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

/* FLASH BANK address */
#define FLASH_BANK1_END_ADDRESS   ((uint32_t)0x807FFFF)

/* Delay definition */
#define EraseTimeout          ((uint32_t)0x000B0000)
#define ProgramTimeout        ((uint32_t)0x00002000)

/**
 * @brief Set flash wait states
 *
 * Note that not all wait states are available on every MCU. See the
 * Flash programming manual for your MCU for restrictions on the
 * allowed value of wait_states for a given system clock (SYSCLK)
 * frequency.
 *
 * @param wait_states number of wait states (one of
 *                    FLASH_WAIT_STATE_0, FLASH_WAIT_STATE_1,
 *                    ..., FLASH_WAIT_STATE_7).
 */
void flash_set_latency(uint32 wait_states) {
    uint32 val = MAPLE_FLASH_BASE->ACR;

    val &= ~MAPLE_FLASH_ACR_LATENCY;
    val |= wait_states;

    MAPLE_FLASH_BASE->ACR = val;
}

/**
 * @brief  Enables or disables the Half cycle flash access.
 * @note   This function can be used for all STM32F10x devices.
 * @param  FLASH_HalfCycleAccess: specifies the FLASH Half cycle Access mode.
 *   This parameter can be one of the following values:
 *     @arg FLASH_HalfCycleAccess_Enable: FLASH Half Cycle Enable
 *     @arg FLASH_HalfCycleAccess_Disable: FLASH Half Cycle Disable
 * @retval None
 */
void flash_halfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess)
{
  /* Check the parameters */
  //  assert_param(IS_FLASH_HALFCYCLEACCESS_STATE(FLASH_HalfCycleAccess));

  /* Enable or disable the Half cycle access */
  MAPLE_FLASH_BASE->ACR &= ACR_HLFCYA_Mask;
  MAPLE_FLASH_BASE->ACR |= FLASH_HalfCycleAccess;
}


/**
 * @brief  Enables or disables the Prefetch Buffer.
 * @note   This function can be used for all STM32F10x devices.
 * @param  FLASH_PrefetchBuffer: specifies the Prefetch buffer status.
 *   This parameter can be one of the following values:
 *     @arg FLASH_PrefetchBuffer_Enable: FLASH Prefetch Buffer Enable
 *     @arg FLASH_PrefetchBuffer_Disable: FLASH Prefetch Buffer Disable
 * @retval None
 */
void flash_prefetchBufferCmd(uint32_t FLASH_PrefetchBuffer)
{
  /* Check the parameters */

  /* Enable or disable the Prefetch Buffer */
  MAPLE_FLASH_BASE->ACR &= ACR_PRFTBE_Mask;
  MAPLE_FLASH_BASE->ACR |= FLASH_PrefetchBuffer;
}


/**
 * @brief  Unlocks the FLASH Program Erase Controller.
 * @note   This function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices this function unlocks Bank1 and Bank2.
 *         - For all other devices it unlocks Bank1 and it is equivalent
 *           to FLASH_UnlockBank1 function..
 * @param  None
 * @retval None
 */
void flash_unlock(void)
{
  /* Authorize the FPEC of Bank1 Access */
  MAPLE_FLASH_BASE->KEYR = FLASH_KEY1;
  MAPLE_FLASH_BASE->KEYR = FLASH_KEY2;

#ifdef STM32F10X_XL
  /* Authorize the FPEC of Bank2 Access */
  MAPLE_FLASH_BASE->KEYR2 = FLASH_KEY1;
  MAPLE_FLASH_BASE->KEYR2 = FLASH_KEY2;
#endif /* STM32F10X_XL */
}



/**
 * @brief  Unlocks the FLASH Bank1 Program Erase Controller.
 * @note   This function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices this function unlocks Bank1.
 *         - For all other devices it unlocks Bank1 and it is
 *           equivalent to FLASH_Unlock function.
 * @param  None
 * @retval None
 */
void flash_unlockBank1(void)
{
  /* Authorize the FPEC of Bank1 Access */
  MAPLE_FLASH_BASE->KEYR = FLASH_KEY1;
  MAPLE_FLASH_BASE->KEYR = FLASH_KEY2;
}

#ifdef STM32F10X_XL
/**
 * @brief  Unlocks the FLASH Bank2 Program Erase Controller.
 * @note   This function can be used only for STM32F10X_XL density devices.
 * @param  None
 * @retval None
 */
void flash_unlockBank2(void)
{
  /* Authorize the FPEC of Bank2 Access */
  MAPLE_FLASH_BASE->KEYR2 = FLASH_KEY1;
  MAPLE_FLASH_BASE->KEYR2 = FLASH_KEY2;

}
#endif /* STM32F10X_XL */

/**
 * @brief  Locks the FLASH Program Erase Controller.
 * @note   This function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices this function Locks Bank1 and Bank2.
 *         - For all other devices it Locks Bank1 and it is equivalent
 *           to FLASH_LockBank1 function.
 * @param  None
 * @retval None
 */
void flash_lock(void)
{
  /* Set the Lock Bit to lock the FPEC and the CR of  Bank1 */
  MAPLE_FLASH_BASE->CR |= CR_LOCK_Set;

#ifdef STM32F10X_XL
  /* Set the Lock Bit to lock the FPEC and the CR of  Bank2 */
  MAPLE_FLASH_BASE->CR2 |= CR_LOCK_Set;
#endif /* STM32F10X_XL */
}

/**
 * @brief  Locks the FLASH Bank1 Program Erase Controller.
 * @note   this function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices this function Locks Bank1.
 *         - For all other devices it Locks Bank1 and it is equivalent
 *           to FLASH_Lock function.
 * @param  None
 * @retval None
 */
void flash_lockBank1(void)
{
  /* Set the Lock Bit to lock the FPEC and the CR of  Bank1 */
  MAPLE_FLASH_BASE->CR |= CR_LOCK_Set;
}

#ifdef STM32F10X_XL
/**
 * @brief  Locks the FLASH Bank2 Program Erase Controller.
 * @note   This function can be used only for STM32F10X_XL density devices.
 * @param  None
 * @retval None
 */
void flash_lockBank2(void)
{
  /* Set the Lock Bit to lock the FPEC and the CR of  Bank2 */
  MAPLE_FLASH_BASE->CR2 |= CR_LOCK_Set;
}
#endif /* STM32F10X_XL */


/**
 * @brief  Erases a specified FLASH page.
 * @note   This function can be used for all STM32F10x devices.
 * @param  Page_Address: The page address to be erased.
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_ErasePage(uint32_t Page_Address)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Page_Address));

#ifdef STM32F10X_XL
  if(Page_Address < FLASH_BANK1_END_ADDRESS)
    {
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastBank1Operation(EraseTimeout);
      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to erase the page */
          FLASH->CR|= CR_PER_Set;
          MAPLE_FLASH_BASE->AR = Page_Address;
          MAPLE_FLASH_BASE->CR|= CR_STRT_Set;

          /* Wait for last operation to be completed */
          status = FLASH_WaitForLastBank1Operation(EraseTimeout);

          /* Disable the PER Bit */
          MAPLE_FLASH_BASE->CR &= CR_PER_Reset;
        }
    }
  else
    {
      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank2Operation(EraseTimeout);
      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to erase the page */
          MAPLE_FLASH_BASE->CR2|= CR_PER_Set;
          MAPLE_FLASH_BASE->AR2 = Page_Address;
          MAPLE_FLASH_BASE->CR2|= CR_STRT_Set;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastBank2Operation(EraseTimeout);

          /* Disable the PER Bit */
          MAPLE_FLASH_BASE->CR2 &= CR_PER_Reset;
        }
    }
#else
  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(EraseTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to erase the page */
      MAPLE_FLASH_BASE->CR|= CR_PER_Set;
      MAPLE_FLASH_BASE->AR = Page_Address;
      MAPLE_FLASH_BASE->CR|= CR_STRT_Set;

      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(EraseTimeout);

      /* Disable the PER Bit */
      MAPLE_FLASH_BASE->CR &= CR_PER_Reset;
    }
#endif /* STM32F10X_XL */

  /* Return the Erase Status */
  return status;
}

/**
 * @brief  Erases all FLASH pages.
 * @note   This function can be used for all STM32F10x devices.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_EraseAllPages(void)
{
  FLASH_Status status = FLASH_COMPLETE;

#ifdef STM32F10X_XL
  /* Wait for last operation to be completed */
  status = flash_WaitForLastBank1Operation(EraseTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to erase all pages */
      MAPLE_FLASH_BASE->CR |= CR_MER_Set;
      MAPLE_FLASH_BASE->CR |= CR_STRT_Set;

      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank1Operation(EraseTimeout);

      /* Disable the MER Bit */
      MAPLE_FLASH_BASE->CR &= CR_MER_Reset;
    }
  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to erase all pages */
      MAPLE_FLASH_BASE->CR2 |= CR_MER_Set;
      MAPLE_FLASH_BASE->CR2 |= CR_STRT_Set;

      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank2Operation(EraseTimeout);

      /* Disable the MER Bit */
      MAPLE_FLASH_BASE->CR2 &= CR_MER_Reset;
    }
#else
  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(EraseTimeout);
  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to erase all pages */
      MAPLE_FLASH_BASE->CR |= CR_MER_Set;
      MAPLE_FLASH_BASE->CR |= CR_STRT_Set;

      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(EraseTimeout);

      /* Disable the MER Bit */
      MAPLE_FLASH_BASE->CR &= CR_MER_Reset;
    }
#endif /* STM32F10X_XL */

  /* Return the Erase Status */
  return status;
}

/**
 * @brief  Erases all Bank1 FLASH pages.
 * @note   This function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices this function erases all Bank1 pages.
 *         - For all other devices it erases all Bank1 pages and it is equivalent
 *           to FLASH_EraseAllPages function.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_EraseAllBank1Pages(void)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Wait for last operation to be completed */
  status = flash_WaitForLastBank1Operation(EraseTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to erase all pages */
      MAPLE_FLASH_BASE->CR |= CR_MER_Set;
      MAPLE_FLASH_BASE->CR |= CR_STRT_Set;

      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank1Operation(EraseTimeout);

      /* Disable the MER Bit */
      MAPLE_FLASH_BASE->CR &= CR_MER_Reset;
    }
  /* Return the Erase Status */
  return status;
}

#ifdef STM32F10X_XL
/**
 * @brief  Erases all Bank2 FLASH pages.
 * @note   This function can be used only for STM32F10x_XL density devices.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_EraseAllBank2Pages(void)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Wait for last operation to be completed */
  status = flash_WaitForLastBank2Operation(EraseTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to erase all pages */
      MAPLE_FLASH_BASE->CR2 |= CR_MER_Set;
      MAPLE_FLASH_BASE->CR2 |= CR_STRT_Set;

      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank2Operation(EraseTimeout);

      /* Disable the MER Bit */
      MAPLE_FLASH_BASE->CR2 &= CR_MER_Reset;
    }
  /* Return the Erase Status */
  return status;
}
#endif /* STM32F10X_XL */

/**
 * @brief  Erases the FLASH option bytes.
 * @note   This functions erases all option bytes except the Read protection (RDP).
 * @note   This function can be used for all STM32F10x devices.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_EraseOptionBytes(void)
{
  uint16_t rdptmp = RDP_Key;

  FLASH_Status status = FLASH_COMPLETE;

  /* Get the actual read protection Option Byte value */
  if(flash_GetReadOutProtectionStatus() != 0)
    {
      rdptmp = 0x00;
    }

  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(EraseTimeout);
  if(status == FLASH_COMPLETE)
    {
      /* Authorize the small information block programming */
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY1;
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY2;

      /* if the previous operation is completed, proceed to erase the option bytes */
      MAPLE_FLASH_BASE->CR |= CR_OPTER_Set;
      MAPLE_FLASH_BASE->CR |= CR_STRT_Set;
      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(EraseTimeout);

      if(status == FLASH_COMPLETE)
        {
          /* if the erase operation is completed, disable the OPTER Bit */
          MAPLE_FLASH_BASE->CR &= CR_OPTER_Reset;

          /* Enable the Option Bytes Programming operation */
          MAPLE_FLASH_BASE->CR |= CR_OPTPG_Set;
          /* Restore the last read protection Option Byte value */
          MAPLE_OB->RDP = (uint16_t)rdptmp;
          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(ProgramTimeout);

          if(status != FLASH_TIMEOUT)
            {
              /* if the program operation is completed, disable the OPTPG Bit */
              MAPLE_FLASH_BASE->CR &= CR_OPTPG_Reset;
            }
        }
      else
        {
          if (status != FLASH_TIMEOUT)
            {
              /* Disable the OPTPG Bit */
              MAPLE_FLASH_BASE->CR &= CR_OPTPG_Reset;
            }
        }
    }
  /* Return the erase status */
  return status;
}

/**
 * @brief  Programs a word at a specified address.
 * @note   This function can be used for all STM32F10x devices.
 * @param  Address: specifies the address to be programmed.
 * @param  Data: specifies the data to be programmed.
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_ProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  volatile uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

#ifdef STM32F10X_XL
  if(Address < FLASH_BANK1_END_ADDRESS - 2)
    {
      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank1Operation(ProgramTimeout);
      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to program the new first
        half word */
          MAPLE_FLASH_BASE->CR |= CR_PG_Set;

          *(__io uint16_t*)Address = (uint16_t)Data;
          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(ProgramTimeout);

          if(status == FLASH_COMPLETE)
            {
              /* if the previous operation is completed, proceed to program the new second
        half word */
              tmp = Address + 2;

              *(__io uint16_t*) tmp = Data >> 16;

              /* Wait for last operation to be completed */
              status = flash_WaitForLastOperation(ProgramTimeout);

              /* Disable the PG Bit */
              MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
            }
          else
            {
              /* Disable the PG Bit */
              MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
            }
        }
    }
  else if(Address == (FLASH_BANK1_END_ADDRESS - 1))
    {
      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank1Operation(ProgramTimeout);

      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to program the new first
        half word */
          MAPLE_FLASH_BASE->CR |= CR_PG_Set;

          *(__io uint16_t*)Address = (uint16_t)Data;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastBank1Operation(ProgramTimeout);

          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
        }
      else
        {
          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
        }

      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank2Operation(ProgramTimeout);

      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to program the new second
      half word */
          MAPLE_FLASH_BASE->CR2 |= CR_PG_Set;
          tmp = Address + 2;

          *(__io uint16_t*) tmp = Data >> 16;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastBank2Operation(ProgramTimeout);

          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR2 &= CR_PG_Reset;
        }
      else
        {
          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR2 &= CR_PG_Reset;
        }
    }
  else
    {
      /* Wait for last operation to be completed */
      status = flash_WaitForLastBank2Operation(ProgramTimeout);

      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to program the new first
        half word */
          MAPLE_FLASH_BASE->CR2 |= CR_PG_Set;

          *(__io uint16_t*)Address = (uint16_t)Data;
          /* Wait for last operation to be completed */
          status = flash_WaitForLastBank2Operation(ProgramTimeout);

          if(status == FLASH_COMPLETE)
            {
              /* if the previous operation is completed, proceed to program the new second
        half word */
              tmp = Address + 2;

              *(__io uint16_t*) tmp = Data >> 16;

              /* Wait for last operation to be completed */
              status = flash_WaitForLastBank2Operation(ProgramTimeout);

              /* Disable the PG Bit */
              MAPLE_FLASH_BASE->CR2 &= CR_PG_Reset;
            }
          else
            {
              /* Disable the PG Bit */
              MAPLE_FLASH_BASE->CR2 &= CR_PG_Reset;
            }
        }
    }
#else
  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(ProgramTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new first
    half word */
      MAPLE_FLASH_BASE->CR |= CR_PG_Set;

      *(__io uint16_t*)Address = (uint16_t)Data;
      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(ProgramTimeout);

      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to program the new second
      half word */
          tmp = Address + 2;

          *(__io uint16_t*) tmp = Data >> 16;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(ProgramTimeout);

          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
        }
      else
        {
          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
        }
    }
#endif /* STM32F10X_XL */

  /* Return the Program Status */
  return status;
}

/**
 * @brief  Programs a half word at a specified address.
 * @note   This function can be used for all STM32F10x devices.
 * @param  Address: specifies the address to be programmed.
 * @param  Data: specifies the data to be programmed.
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

#ifdef STM32F10X_XL
  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(ProgramTimeout);

  if(Address < FLASH_BANK1_END_ADDRESS)
    {
      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to program the new data */
          MAPLE_FLASH_BASE->CR |= CR_PG_Set;

          *(__io uint16_t*)Address = Data;
          /* Wait for last operation to be completed */
          status = flash_WaitForLastBank1Operation(ProgramTimeout);

          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
        }
    }
  else
    {
      if(status == FLASH_COMPLETE)
        {
          /* if the previous operation is completed, proceed to program the new data */
          MAPLE_FLASH_BASE->CR2 |= CR_PG_Set;

          *(__io uint16_t*)Address = Data;
          /* Wait for last operation to be completed */
          status = flash_WaitForLastBank2Operation(ProgramTimeout);

          /* Disable the PG Bit */
          MAPLE_FLASH_BASE->CR2 &= CR_PG_Reset;
        }
    }
#else
  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(ProgramTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new data */
      MAPLE_FLASH_BASE->CR |= CR_PG_Set;

      *(__io uint16_t*)Address = Data;
      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(ProgramTimeout);

      /* Disable the PG Bit */
      MAPLE_FLASH_BASE->CR &= CR_PG_Reset;
    }
#endif  /* STM32F10X_XL */

  /* Return the Program Status */
  return status;
}

/**
 * @brief  Programs a half word at a specified Option Byte Data address.
 * @note   This function can be used for all STM32F10x devices.
 * @param  Address: specifies the address to be programmed.
 *   This parameter can be 0x1FFFF804 or 0x1FFFF806.
 * @param  Data: specifies the data to be programmed.
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_ProgramOptionByteData(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Check the parameters */
  assert_param(IS_OB_DATA_ADDRESS(Address));
  status = flash_WaitForLastOperation(ProgramTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* Authorize the small information block programming */
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY1;
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY2;
      /* Enables the Option Bytes Programming operation */
      MAPLE_FLASH_BASE->CR |= CR_OPTPG_Set;
      *(__io uint16_t*)Address = Data;

      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(ProgramTimeout);
      if(status != FLASH_TIMEOUT)
        {
          /* if the program operation is completed, disable the OPTPG Bit */
          MAPLE_FLASH_BASE->CR &= CR_OPTPG_Reset;
        }
    }
  /* Return the Option Byte Data Program Status */
  return status;
}

/**
 * @brief  Write protects the desired pages
 * @note   This function can be used for all STM32F10x devices.
 * @param  FLASH_Pages: specifies the address of the pages to be write protected.
 *   This parameter can be:
 *     @arg For @b STM32_Low-density_devices: value between FLASH_WRProt_Pages0to3 and FLASH_WRProt_Pages28to31
 *     @arg For @b STM32_Medium-density_devices: value between FLASH_WRProt_Pages0to3
 *       and FLASH_WRProt_Pages124to127
 *     @arg For @b STM32_High-density_devices: value between FLASH_WRProt_Pages0to1 and
 *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to255
 *     @arg For @b STM32_Connectivity_line_devices: value between FLASH_WRProt_Pages0to1 and
 *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to127
 *     @arg For @b STM32_XL-density_devices: value between FLASH_WRProt_Pages0to1 and
 *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to511
 *     @arg FLASH_WRProt_AllPages
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_EnableWriteProtection(uint32_t FLASH_Pages)
{
  uint16_t WRP0_Data = 0xFFFF, WRP1_Data = 0xFFFF, WRP2_Data = 0xFFFF, WRP3_Data = 0xFFFF;

  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_WRPROT_PAGE(FLASH_Pages));

  FLASH_Pages = (uint32_t)(~FLASH_Pages);
  WRP0_Data = (uint16_t)(FLASH_Pages & WRP0_Mask);
  WRP1_Data = (uint16_t)((FLASH_Pages & WRP1_Mask) >> 8);
  WRP2_Data = (uint16_t)((FLASH_Pages & WRP2_Mask) >> 16);
  WRP3_Data = (uint16_t)((FLASH_Pages & WRP3_Mask) >> 24);

  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(ProgramTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* Authorizes the small information block programming */
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY1;
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY2;
      MAPLE_FLASH_BASE->CR |= CR_OPTPG_Set;
      if(WRP0_Data != 0xFF)
        {
          MAPLE_OB->WRP0 = WRP0_Data;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(ProgramTimeout);
        }
      if((status == FLASH_COMPLETE) && (WRP1_Data != 0xFF))
        {
          MAPLE_OB->WRP1 = WRP1_Data;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(ProgramTimeout);
        }
      if((status == FLASH_COMPLETE) && (WRP2_Data != 0xFF))
        {
          MAPLE_OB->WRP2 = WRP2_Data;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(ProgramTimeout);
        }

      if((status == FLASH_COMPLETE)&& (WRP3_Data != 0xFF))
        {
          MAPLE_OB->WRP3 = WRP3_Data;

          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(ProgramTimeout);
        }

      if(status != FLASH_TIMEOUT)
        {
          /* if the program operation is completed, disable the OPTPG Bit */
          MAPLE_FLASH_BASE->CR &= CR_OPTPG_Reset;
        }
    }
  /* Return the write protection operation Status */
  return status;
}

/**
 * @brief  Enables or disables the read out protection.
 * @note   If the user has already programmed the other option bytes before calling
 *   this function, he must re-program them since this function erases all option bytes.
 * @note   This function can be used for all STM32F10x devices.
 * @param  Newstate: new state of the ReadOut Protection.
 *   This parameter can be: ENABLE or DISABLE.
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_ReadOutProtection(uint8_t NewState)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Check the parameters */

  status = flash_WaitForLastOperation(EraseTimeout);
  if(status == FLASH_COMPLETE)
    {
      /* Authorizes the small information block programming */
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY1;
      MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY2;
      MAPLE_FLASH_BASE->CR |= CR_OPTER_Set;
      MAPLE_FLASH_BASE->CR |= CR_STRT_Set;
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(EraseTimeout);
      if(status == FLASH_COMPLETE)
        {
          /* if the erase operation is completed, disable the OPTER Bit */
          MAPLE_FLASH_BASE->CR &= CR_OPTER_Reset;
          /* Enable the Option Bytes Programming operation */
          MAPLE_FLASH_BASE->CR |= CR_OPTPG_Set;
          if(NewState != 0)
            {
              MAPLE_OB->RDP = 0x00;
            }
          else
            {
              MAPLE_OB->RDP = RDP_Key;
            }
          /* Wait for last operation to be completed */
          status = flash_WaitForLastOperation(EraseTimeout);

          if(status != FLASH_TIMEOUT)
            {
              /* if the program operation is completed, disable the OPTPG Bit */
              MAPLE_FLASH_BASE->CR &= CR_OPTPG_Reset;
            }
        }
      else
        {
          if(status != FLASH_TIMEOUT)
            {
              /* Disable the OPTER Bit */
              MAPLE_FLASH_BASE->CR &= CR_OPTER_Reset;
            }
        }
    }
  /* Return the protection operation Status */
  return status;
}

/**
 * @brief  Programs the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY.
 * @note   This function can be used for all STM32F10x devices.
 * @param  OB_IWDG: Selects the IWDG mode
 *   This parameter can be one of the following values:
 *     @arg OB_IWDG_SW: Software IWDG selected
 *     @arg OB_IWDG_HW: Hardware IWDG selected
 * @param  OB_STOP: Reset event when entering STOP mode.
 *   This parameter can be one of the following values:
 *     @arg OB_STOP_NoRST: No reset generated when entering in STOP
 *     @arg OB_STOP_RST: Reset generated when entering in STOP
 * @param  OB_STDBY: Reset event when entering Standby mode.
 *   This parameter can be one of the following values:
 *     @arg OB_STDBY_NoRST: No reset generated when entering in STANDBY
 *     @arg OB_STDBY_RST: Reset generated when entering in STANDBY
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */


  /* Authorize the small information block programming */
  MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY1;
  MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY2;

  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(ProgramTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* Enable the Option Bytes Programming operation */
      MAPLE_FLASH_BASE->CR |= CR_OPTPG_Set;

      MAPLE_OB->USER = OB_IWDG | (uint16_t)(OB_STOP | (uint16_t)(OB_STDBY | ((uint16_t)0xF8)));

      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(ProgramTimeout);
      if(status != FLASH_TIMEOUT)
        {
          /* if the program operation is completed, disable the OPTPG Bit */
          MAPLE_FLASH_BASE->CR &= CR_OPTPG_Reset;
        }
    }
  /* Return the Option Byte program Status */
  return status;
}

#ifdef STM32F10X_XL
/**
 * @brief  Configures to boot from Bank1 or Bank2.
 * @note   This function can be used only for STM32F10x_XL density devices.
 * @param  FLASH_BOOT: select the FLASH Bank to boot from.
 *   This parameter can be one of the following values:
 *     @arg FLASH_BOOT_Bank1: At startup, if boot pins are set in boot from user Flash
 *        position and this parameter is selected the device will boot from Bank1(Default).
 *     @arg FLASH_BOOT_Bank2: At startup, if boot pins are set in boot from user Flash
 *        position and this parameter is selected the device will boot from Bank2 or Bank1,
 *        depending on the activation of the bank. The active banks are checked in
 *        the following order: Bank2, followed by Bank1.
 *        The active bank is recognized by the value programmed at the base address
 *        of the respective bank (corresponding to the initial stack pointer value
 *        in the interrupt vector table).
 *        For more information, please refer to AN2606 from www.st.com.
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status flash_BootConfig(uint16_t FLASH_BOOT)
{
  FLASH_Status status = FLASH_COMPLETE;
  assert_param(IS_FLASH_BOOT(FLASH_BOOT));
  /* Authorize the small information block programming */
  MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY1;
  MAPLE_FLASH_BASE->OPTKEYR = FLASH_KEY2;

  /* Wait for last operation to be completed */
  status = flash_WaitForLastOperation(ProgramTimeout);

  if(status == FLASH_COMPLETE)
    {
      /* Enable the Option Bytes Programming operation */
      MAPLE_FLASH_BASE->CR |= CR_OPTPG_Set;

      if(FLASH_BOOT == FLASH_BOOT_Bank1)
        {
          MAPLE_OB->USER |= OB_USER_BFB2;
        }
      else
        {
          MAPLE_OB->USER &= (uint16_t)(~(uint16_t)(OB_USER_BFB2));
        }
      /* Wait for last operation to be completed */
      status = flash_WaitForLastOperation(ProgramTimeout);
      if(status != FLASH_TIMEOUT)
        {
          /* if the program operation is completed, disable the OPTPG Bit */
          MAPLE_FLASH_BASE->CR &= CR_OPTPG_Reset;
        }
    }
  /* Return the Option Byte program Status */
  return status;
}
#endif /* STM32F10X_XL */

/**
 * @brief  Returns the FLASH User Option Bytes values.
 * @note   This function can be used for all STM32F10x devices.
 * @param  None
 * @retval The FLASH User Option Bytes values:IWDG_SW(Bit0), RST_STOP(Bit1)
 *         and RST_STDBY(Bit2).
 */
uint32_t FLASH_GetUserOptionByte(void)
{
  /* Return the User Option Byte */
  return (uint32_t)(MAPLE_FLASH_BASE->OBR >> 2);
}

/**
 * @brief  Returns the FLASH Write Protection Option Bytes Register value.
 * @note   This function can be used for all STM32F10x devices.
 * @param  None
 * @retval The FLASH Write Protection  Option Bytes Register value
 */
uint32_t flash_GetWriteProtectionOptionByte(void)
{
  /* Return the Flash write protection Register value */
  return (uint32_t)(MAPLE_FLASH_BASE->WRPR);
}

/**
 * @brief  Checks whether the FLASH Read Out Protection Status is set or not.
 * @note   This function can be used for all STM32F10x devices.
 * @param  None
 * @retval FLASH ReadOut Protection Status(SET or RESET)
 */
uint8_t flash_GetReadOutProtectionStatus(void)
{
  uint8_t readoutstatus = 0;
  if ((MAPLE_FLASH_BASE->OBR & RDPRT_Mask) != (uint32_t)0)
    {
      readoutstatus = 1;
    }
  else
    {
      readoutstatus = 1;
    }
  return readoutstatus;
}

/**
 * @brief  Checks whether the FLASH Prefetch Buffer status is set or not.
 * @note   This function can be used for all STM32F10x devices.
 * @param  None
 * @retval FLASH Prefetch Buffer Status (SET or RESET).
 */
uint8_t flash_GetPrefetchBufferStatus(void)
{
  uint8_t bitstatus = 0;

  if ((MAPLE_FLASH_BASE->ACR & ACR_PRFTBS_Mask) != (uint32_t)0)
    {
      bitstatus = 1;
    }
  else
    {
      bitstatus = 0;
    }
  /* Return the new state of FLASH Prefetch Buffer Status (SET or RESET) */
  return bitstatus;
}

/**
 * @brief  Enables or disables the specified FLASH interrupts.
 * @note   This function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices, enables or disables the specified FLASH interrupts
              for Bank1 and Bank2.
 *         - For other devices it enables or disables the specified FLASH interrupts for Bank1.
 * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or disabled.
 *   This parameter can be any combination of the following values:
 *     @arg FLASH_IT_ERROR: FLASH Error Interrupt
 *     @arg FLASH_IT_EOP: FLASH end of operation Interrupt
 * @param  NewState: new state of the specified Flash interrupts.
 *   This parameter can be: ENABLE or DISABLE.
 * @retval None
 */
void flash_ITConfig(uint32_t FLASH_IT, uint8_t NewState)
{
#ifdef STM32F10X_XL
  /* Check the parameters */


  if((FLASH_IT & 0x80000000) != 0x0)
    {
      if(NewState != DISABLE)
        {
          /* Enable the interrupt sources */
          MAPLE_FLASH_BASE->CR2 |= (FLASH_IT & 0x7FFFFFFF);
        }
      else
        {
          /* Disable the interrupt sources */
          MAPLE_FLASH_BASE->CR2 &= ~(uint32_t)(FLASH_IT & 0x7FFFFFFF);
        }
    }
  else
    {
      if(NewState != DISABLE)
        {
          /* Enable the interrupt sources */
          MAPLE_FLASH_BASE->CR |= FLASH_IT;
        }
      else
        {
          /* Disable the interrupt sources */
          MAPLE_FLASH_BASE->CR &= ~(uint32_t)FLASH_IT;
        }
    }
#else
  /* Check the parameters */


  if(NewState != 0)
    {
      /* Enable the interrupt sources */
      MAPLE_FLASH_BASE->CR |= FLASH_IT;
    }
  else
    {
      /* Disable the interrupt sources */
      MAPLE_FLASH_BASE->CR &= ~(uint32_t)FLASH_IT;
    }
#endif /* STM32F10X_XL */
}

/**
 * @brief  Checks whether the specified FLASH flag is set or not.
 * @note   This function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices, this function checks whether the specified
 *           Bank1 or Bank2 flag is set or not.
 *         - For other devices, it checks whether the specified Bank1 flag is
 *           set or not.
 * @param  FLASH_FLAG: specifies the FLASH flag to check.
 *   This parameter can be one of the following values:
 *     @arg FLASH_FLAG_BSY: FLASH Busy flag
 *     @arg FLASH_FLAG_PGERR: FLASH Program error flag
 *     @arg FLASH_FLAG_WRPRTERR: FLASH Write protected error flag
 *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag
 *     @arg FLASH_FLAG_OPTERR:  FLASH Option Byte error flag
 * @retval The new state of FLASH_FLAG (SET or RESET).
 */
uint8_t flash_GetFlagStatus(uint32_t FLASH_FLAG)
{
  uint8_t bitstatus = 0;

#ifdef STM32F10X_XL
  /* Check the parameters */

  if(FLASH_FLAG == FLASH_FLAG_OPTERR)
    {
      if((MAPLE_FLASH_BASE->OBR & FLASH_FLAG_OPTERR) != (uint32_t)RESET)
        {
          bitstatus = SET;
        }
      else
        {
          bitstatus = RESET;
        }
    }
  else
    {
      if((FLASH_FLAG & 0x80000000) != 0x0)
        {
          if((MAPLE_FLASH_BASE->SR2 & FLASH_FLAG) != (uint32_t)RESET)
            {
              bitstatus = SET;
            }
          else
            {
              bitstatus = RESET;
            }
        }
      else
        {
          if((MAPLE_FLASH_BASE->SR & FLASH_FLAG) != (uint32_t)RESET)
            {
              bitstatus = SET;
            }
          else
            {
              bitstatus = RESET;
            }
        }
    }
#else
  /* Check the parameters */

  if(FLASH_FLAG == FLASH_FLAG_OPTERR)
    {
      if((MAPLE_FLASH_BASE->OBR & FLASH_FLAG_OPTERR) != (uint32_t)0)
        {
          bitstatus = 1;
        }
      else
        {
          bitstatus = 0;
        }
    }
  else
    {
      if((MAPLE_FLASH_BASE->SR & FLASH_FLAG) != (uint32_t)0)
        {
          bitstatus = 1;
        }
      else
        {
          bitstatus = 0;
        }
    }
#endif /* STM32F10X_XL */

  /* Return the new state of FLASH_FLAG (SET or RESET) */
  return bitstatus;
}

/**
 * @brief  Clears the FLASH's pending flags.
 * @note   This function can be used for all STM32F10x devices.
 *         - For STM32F10X_XL devices, this function clears Bank1 or Bank2’s pending flags
 *         - For other devices, it clears Bank1’s pending flags.
 * @param  FLASH_FLAG: specifies the FLASH flags to clear.
 *   This parameter can be any combination of the following values:
 *     @arg FLASH_FLAG_PGERR: FLASH Program error flag
 *     @arg FLASH_FLAG_WRPRTERR: FLASH Write protected error flag
 *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag
 * @retval None
 */
void flash_ClearFlag(uint32_t FLASH_FLAG)
{
#ifdef STM32F10X_XL
  /* Check the parameters */


  if((FLASH_FLAG & 0x80000000) != 0x0)
    {
      /* Clear the flags */
      MAPLE_FLASH_BASE->SR2 = FLASH_FLAG;
    }
  else
    {
      /* Clear the flags */
      MAPLE_FLASH_BASE->SR = FLASH_FLAG;
    }

#else
  /* Check the parameters */


  /* Clear the flags */
  MAPLE_FLASH_BASE->SR = FLASH_FLAG;
#endif /* STM32F10X_XL */
}

/**
 * @brief  Returns the FLASH Status.
 * @note   This function can be used for all STM32F10x devices, it is equivalent
 *         to FLASH_GetBank1Status function.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP or FLASH_COMPLETE
 */
uint8_t flash_GetStatus(void)
{
  uint8_t flashstatus = FLASH_COMPLETE;

  if((MAPLE_FLASH_BASE->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY)
    {
      flashstatus = FLASH_BUSY;
    }
  else
    {
      if((MAPLE_FLASH_BASE->SR & FLASH_FLAG_PGERR) != 0)
        {
          flashstatus = FLASH_ERROR_PG;
        }
      else
        {
          if((MAPLE_FLASH_BASE->SR & FLASH_FLAG_WRPRTERR) != 0 )
            {
              flashstatus = FLASH_ERROR_WRP;
            }
          else
            {
              flashstatus = FLASH_COMPLETE;
            }
        }
    }
  /* Return the Flash Status */
  return flashstatus;
}

/**
 * @brief  Returns the FLASH Bank1 Status.
 * @note   This function can be used for all STM32F10x devices, it is equivalent
 *         to FLASH_GetStatus function.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP or FLASH_COMPLETE
 */
uint8_t flash_GetBank1Status(void)
{
  uint8_t flashstatus = FLASH_COMPLETE;

  if((MAPLE_FLASH_BASE->SR & FLASH_FLAG_BANK1_BSY) == FLASH_FLAG_BSY)
    {
      flashstatus = FLASH_BUSY;
    }
  else
    {
      if((MAPLE_FLASH_BASE->SR & FLASH_FLAG_BANK1_PGERR) != 0)
        {
          flashstatus = FLASH_ERROR_PG;
        }
      else
        {
          if((MAPLE_FLASH_BASE->SR & FLASH_FLAG_BANK1_WRPRTERR) != 0 )
            {
              flashstatus = FLASH_ERROR_WRP;
            }
          else
            {
              flashstatus = FLASH_COMPLETE;
            }
        }
    }
  /* Return the Flash Status */
  return flashstatus;
}

#ifdef STM32F10X_XL
/**
 * @brief  Returns the FLASH Bank2 Status.
 * @note   This function can be used for STM32F10x_XL density devices.
 * @param  None
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
 *        FLASH_ERROR_WRP or FLASH_COMPLETE
 */
uint8_t flash_GetBank2Status(void)
{
  uint8_t flashstatus = FLASH_COMPLETE;

  if((MAPLE_FLASH_BASE->SR2 & (FLASH_FLAG_BANK2_BSY & 0x7FFFFFFF)) == (FLASH_FLAG_BANK2_BSY & 0x7FFFFFFF))
    {
      flashstatus = FLASH_BUSY;
    }
  else
    {
      if((MAPLE_FLASH_BASE->SR2 & (FLASH_FLAG_BANK2_PGERR & 0x7FFFFFFF)) != 0)
        {
          flashstatus = FLASH_ERROR_PG;
        }
      else
        {
          if((MAPLE_FLASH_BASE->SR2 & (FLASH_FLAG_BANK2_WRPRTERR & 0x7FFFFFFF)) != 0 )
            {
              flashstatus = FLASH_ERROR_WRP;
            }
          else
            {
              flashstatus = FLASH_COMPLETE;
            }
        }
    }
  /* Return the Flash Status */
  return flashstatus;
}
#endif /* STM32F10X_XL */
/**
 * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
 * @note   This function can be used for all STM32F10x devices,
 *         it is equivalent to FLASH_WaitForLastBank1Operation.
 *         - For STM32F10X_XL devices this function waits for a Bank1 Flash operation
 *           to complete or a TIMEOUT to occur.
 *         - For all other devices it waits for a Flash operation to complete
 *           or a TIMEOUT to occur.
 * @param  Timeout: FLASH programming Timeout
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
uint8_t flash_WaitForLastOperation(uint32_t Timeout)
{
  uint8_t status = FLASH_COMPLETE;

  /* Check for the Flash Status */
  status = flash_GetBank1Status();
  /* Wait for a Flash operation to complete or a TIMEOUT to occur */
  while((status == FLASH_BUSY) && (Timeout != 0x00))
    {
      status = flash_GetBank1Status();
      Timeout--;
    }
  if(Timeout == 0x00 )
    {
      status = FLASH_TIMEOUT;
    }
  /* Return the operation status */
  return status;
}

/**
 * @brief  Waits for a Flash operation on Bank1 to complete or a TIMEOUT to occur.
 * @note   This function can be used for all STM32F10x devices,
 *         it is equivalent to FLASH_WaitForLastOperation.
 * @param  Timeout: FLASH programming Timeout
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
uint8_t flash_WaitForLastBank1Operation(uint32_t Timeout)
{
  uint8_t status = FLASH_COMPLETE;

  /* Check for the Flash Status */
  status = FLASH_GetBank1Status();
  /* Wait for a Flash operation to complete or a TIMEOUT to occur */
  while((status == FLASH_FLAG_BANK1_BSY) && (Timeout != 0x00))
    {
      status = flash_GetBank1Status();
      Timeout--;
    }
  if(Timeout == 0x00 )
    {
      status = FLASH_TIMEOUT;
    }
  /* Return the operation status */
  return status;
}

#ifdef STM32F10X_XL
/**
 * @brief  Waits for a Flash operation on Bank2 to complete or a TIMEOUT to occur.
 * @note   This function can be used only for STM32F10x_XL density devices.
 * @param  Timeout: FLASH programming Timeout
 * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
 *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
uint8_t flash_WaitForLastBank2Operation(uint32_t Timeout)
{
  uint8_t status = FLASH_COMPLETE;

  /* Check for the Flash Status */
  status = flash_GetBank2Status();
  /* Wait for a Flash operation to complete or a TIMEOUT to occur */
  while((status == (FLASH_FLAG_BANK2_BSY & 0x7FFFFFFF)) && (Timeout != 0x00))
    {
      status = flash_GetBank2Status();
      Timeout--;
    }
  if(Timeout == 0x00 )
    {
      status = FLASH_TIMEOUT;
    }
  /* Return the operation status */
  return status;
}
#endif /* STM32F10X_XL */



/* Private define ------------------------------------------------------------*/
/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
#define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#else
#define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#endif

#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08008000)
#define BANK1_WRITE_END_ADDR    ((uint32_t)0x0800C000)

#ifdef STM32F10X_XL
#define BANK2_WRITE_START_ADDR   ((uint32_t)0x08088000)
#define BANK2_WRITE_END_ADDR     ((uint32_t)0x0808C000)
#endif /* STM32F10X_XL */
/*******************************************************************************
 * Function Name  : FLASH_If_Erase
 * Description    : Erase sector
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
uint16_t flash_if_Erase(uint32_t SectorAddress)
{
  uint32_t EraseCounter = 0x00, Address = 0x00;
  uint32_t NbrOfPage = 0x00;
  volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
  /* Unlock the Flash Bank1 Program Erase controller */
  FLASH_UnlockBank1();

  /* Define the number of page to be erased */
  NbrOfPage = (BANK1_WRITE_END_ADDR - BANK1_WRITE_START_ADDR) / FLASH_PAGE_SIZE;

  /* Clear All pending flags */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

  /* Erase the FLASH pages */
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
      FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
    }

  FLASH_LockBank1();

  return 1;
}

/*******************************************************************************
 * Function Name  : FLASH_If_Write
 * Description    : Write sectors
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
uint16_t flash_if_Write(uint32_t SectorAddress,uint32_t MAL_Buffer[], uint32_t DataLength)
{
  uint32_t idx = 0;
  uint32_t EraseCounter = 0x00, Address = 0x00;
  volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
  /* Program Flash Bank1 */
  FLASH_UnlockBank1();

  Address = BANK1_WRITE_START_ADDR;

  while((Address < BANK1_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
    {
      FLASHStatus = FLASH_ProgramWord(Address, MAL_Buffer[Address]);
      Address = Address + 4;
    }

  FLASH_LockBank1();
  return 1;
}

/*******************************************************************************
 * Function Name  : FLASH_If_Read
 * Description    : Read sectors
 * Input          : None
 * Output         : None
 * Return         : buffer address pointer
 *******************************************************************************/
uint8_t *flash_if_Read (uint32_t SectorAddress, uint32_t DataLength)
{
  return  (uint8_t*)(SectorAddress);
}
