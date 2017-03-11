/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * @file libmaple/include/libmaple/usart.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>,
 *         Perry Hung <perry@leaflabs.com>
 * @brief USART definitions and prototypes
 */

#ifndef _LIBMAPLE_USART_H_
#define _LIBMAPLE_USART_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <libmaple/libmaple_types.h>
#include <libmaple/util.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
#include <libmaple/ring_buffer.h>

 /* Roger clark. Replaced with line below #include <series/usart.h>*/
#include "system/usart.h"

/*
 * Register map (common across supported STM32 series).
 */

/** USART register map type */
typedef struct usart_reg_map {
    __io uint32 SR;             /**< Status register */
    __io uint32 DR;             /**< Data register */
    __io uint32 BRR;            /**< Baud rate register */
    __io uint32 CR1;            /**< Control register 1 */
    __io uint32 CR2;            /**< Control register 2 */
    __io uint32 CR3;            /**< Control register 3 */
    __io uint32 GTPR;           /**< Guard time and prescaler register */
} usart_reg_map;

/*
 * Register bit definitions
 */

/* Status register */

/** Clear to send bit */
#define MAPLE_USART_SR_CTS_BIT                9
/** Line break detection bit */
#define MAPLE_USART_SR_LBD_BIT                8
/** Transmit data register empty bit */
#define MAPLE_USART_SR_TXE_BIT                7
/** Transmission complete bit */
#define MAPLE_USART_SR_TC_BIT                 6
/** Read data register not empty bit */
#define MAPLE_USART_SR_RXNE_BIT               5
/** IDLE line detected bit */
#define MAPLE_USART_SR_IDLE_BIT               4
/** Overrun error bit */
#define MAPLE_USART_SR_ORE_BIT                3
/** Noise error bit */
#define MAPLE_USART_SR_NE_BIT                 2
/**
 * @brief Synonym for MAPLE_USART_SR_NE_BIT.
 *
 * Some series (e.g. STM32F2) use "NF" for "noise flag" instead of the
 * original "NE" for "noise error". The meaning of the bit is
 * unchanged, but the NF flag can be disabled when the line is
 * noise-free.
 *
 * @see MAPLE_USART_SR_NE_BIT
 */
#define MAPLE_USART_SR_NF_BIT                 MAPLE_USART_SR_NE_BIT
/** Framing error bit */
#define MAPLE_USART_SR_FE_BIT                 1
/** Parity error bit */
#define MAPLE_USART_SR_PE_BIT                 0

/** Clear to send mask */
#define MAPLE_USART_SR_CTS                    BIT(MAPLE_USART_SR_CTS_BIT)
/** Line break detected mask */
#define MAPLE_USART_SR_LBD                    BIT(MAPLE_USART_SR_LBD_BIT)
/** Transmit data register empty mask */
#define MAPLE_USART_SR_TXE                    BIT(MAPLE_USART_SR_TXE_BIT)
/** Transmission complete mask */
#define MAPLE_USART_SR_TC                     BIT(MAPLE_USART_SR_TC_BIT)
/** Read data register not empty mask */
#define MAPLE_USART_SR_RXNE                   BIT(MAPLE_USART_SR_RXNE_BIT)
/** IDLE line detected mask */
#define MAPLE_USART_SR_IDLE                   BIT(MAPLE_USART_SR_IDLE_BIT)
/** Overrun error mask */
#define MAPLE_USART_SR_ORE                    BIT(MAPLE_USART_SR_ORE_BIT)
/** Noise error mask */
#define MAPLE_USART_SR_NE                     BIT(MAPLE_USART_SR_NE_BIT)
/**
 * @brief Synonym for MAPLE_USART_SR_NE.
 * @see MAPLE_USART_SR_NF_BIT
 */
#define MAPLE_USART_SR_NF                     MAPLE_USART_SR_NE
/** Framing error mask */
#define MAPLE_USART_SR_FE                     BIT(MAPLE_USART_SR_FE_BIT)
/** Parity error mask */
#define MAPLE_USART_SR_PE                     BIT(MAPLE_USART_SR_PE_BIT)

/* Data register */

/** Data register data value mask */
#define MAPLE_USART_DR_DR                     0xFF

/* Baud rate register */

/** Mantissa of MAPLE_USARTDIV mask */
#define MAPLE_USART_BRR_DIV_MANTISSA          (0xFFF << 4)
/** Fraction of MAPLE_USARTDIV mask */
#define MAPLE_USART_BRR_DIV_FRACTION          0xF

/* Control register 1 */

/** MAPLE_USART enable bit */
#define MAPLE_USART_CR1_UE_BIT                13
/** Word length bit */
#define MAPLE_USART_CR1_M_BIT                 12
/** Wakeup method bit */
#define MAPLE_USART_CR1_WAKE_BIT              11
/** Parity control enable bit */
#define MAPLE_USART_CR1_PCE_BIT               10
/** Parity selection bit */
#define MAPLE_USART_CR1_PS_BIT                9
/** Parity error interrupt enable bit */
#define MAPLE_USART_CR1_PEIE_BIT              8
/** Transmit data regsiter not empty interrupt enable bit */
#define MAPLE_USART_CR1_TXEIE_BIT             7
/** Transmission complete interrupt enable bit */
#define MAPLE_USART_CR1_TCIE_BIT              6
/** RXNE interrupt enable bit */
#define MAPLE_USART_CR1_RXNEIE_BIT            5
/** IDLE interrupt enable bit */
#define MAPLE_USART_CR1_IDLEIE_BIT            4
/** Transmitter enable bit */
#define MAPLE_USART_CR1_TE_BIT                3
/** Receiver enable bit */
#define MAPLE_USART_CR1_RE_BIT                2
/** Receiver wakeup bit */
#define MAPLE_USART_CR1_RWU_BIT               1
/** Send break bit */
#define MAPLE_USART_CR1_SBK_BIT               0

/** MAPLE_USART enable mask */
#define MAPLE_USART_CR1_UE                    BIT(MAPLE_USART_CR1_UE_BIT)
/** Word length mask */
#define MAPLE_USART_CR1_M                     BIT(MAPLE_USART_CR1_M_BIT)
/** Word length: 1 start bit, 8 data bits, n stop bit */
#define MAPLE_USART_CR1_M_8N1                 (0 << MAPLE_USART_CR1_M_BIT)
/** Word length: 1 start bit, 9 data bits, n stop bit */
#define MAPLE_USART_CR1_M_9N1                 (1 << MAPLE_USART_CR1_M_BIT)
/** Wakeup method mask */
#define MAPLE_USART_CR1_WAKE                  BIT(MAPLE_USART_CR1_WAKE_BIT)
/** Wakeup on idle line */
#define MAPLE_USART_CR1_WAKE_IDLE             (0 << MAPLE_USART_CR1_WAKE_BIT)
/** Wakeup on address mark */
#define MAPLE_USART_CR1_WAKE_ADDR             (1 << MAPLE_USART_CR1_WAKE_BIT)
/** Parity control enable mask */
#define MAPLE_USART_CR1_PCE                   BIT(MAPLE_USART_CR1_PCE_BIT)
/** Parity selection mask */
#define MAPLE_USART_CR1_PS                    BIT(MAPLE_USART_CR1_PS_BIT)
/** Parity selection: even parity */
#define MAPLE_USART_CR1_PS_EVEN               (0 << MAPLE_USART_CR1_PS_BIT)
/** Parity selection: odd parity */
#define MAPLE_USART_CR1_PS_ODD                (1 << MAPLE_USART_CR1_PS_BIT)
/** Parity error interrupt enable mask */
#define MAPLE_USART_CR1_PEIE                  BIT(MAPLE_USART_CR1_PEIE_BIT)
/** Transmit data register empty interrupt enable mask */
#define MAPLE_USART_CR1_TXEIE                 BIT(MAPLE_USART_CR1_TXEIE_BIT)
/** Transmission complete interrupt enable mask */
#define MAPLE_USART_CR1_TCIE                  BIT(MAPLE_USART_CR1_TCIE_BIT)
/** RXNE interrupt enable mask */
#define MAPLE_USART_CR1_RXNEIE                BIT(MAPLE_USART_CR1_RXNEIE_BIT)
/** IDLE line interrupt enable mask */
#define MAPLE_USART_CR1_IDLEIE                BIT(MAPLE_USART_CR1_IDLEIE_BIT)
/** Transmitter enable mask */
#define MAPLE_USART_CR1_TE                    BIT(MAPLE_USART_CR1_TE_BIT)
/** Receiver enable mask */
#define MAPLE_USART_CR1_RE                    BIT(MAPLE_USART_CR1_RE_BIT)
/** Receiver wakeup mask */
#define MAPLE_USART_CR1_RWU                   BIT(MAPLE_USART_CR1_RWU_BIT)
/** Receiver wakeup: receiver in active mode */
#define MAPLE_USART_CR1_RWU_ACTIVE            (0 << MAPLE_USART_CR1_RWU_BIT)
/** Receiver wakeup: receiver in mute mode */
#define MAPLE_USART_CR1_RWU_MUTE              (1 << MAPLE_USART_CR1_RWU_BIT)
/** Send break */
#define MAPLE_USART_CR1_SBK                   BIT(MAPLE_USART_CR1_SBK_BIT)

/* Control register 2 */

/** LIN mode enable bit */
#define MAPLE_USART_CR2_LINEN_BIT             14
/** Clock enable bit */
#define MAPLE_USART_CR2_CLKEN_BIT             11
/** Clock polarity bit */
#define MAPLE_USART_CR2_CPOL_BIT              10
/** Clock phase bit */
#define MAPLE_USART_CR2_CPHA_BIT              9
/** Last bit clock pulse bit */
#define MAPLE_USART_CR2_LBCL_BIT              8
/** LIN break detection interrupt enable bit */
#define MAPLE_USART_CR2_LBDIE_BIT             6
/** LIN break detection length bit */
#define MAPLE_USART_CR2_LBDL_BIT              5

/** LIN mode enable mask */
#define MAPLE_USART_CR2_LINEN                 BIT(MAPLE_USART_CR2_LINEN_BIT)
/** STOP bits mask */
#define MAPLE_USART_CR2_STOP                  (0x3 << 12)
/** STOP bits: 1 stop bit */
#define MAPLE_USART_CR2_STOP_BITS_1           (0x0 << 12)
/**
 * @brief STOP bits: 0.5 stop bits
 * Not available  on UART4, UART5. */
#define MAPLE_USART_CR2_STOP_BITS_POINT_5     (0x1 << 12)
/** STOP bits: 2 stop bits */
#define MAPLE_USART_CR2_STOP_BITS_2           (0x2 << 12)
/**
 * @brief STOP bits: 1.5 stop bits
 * Not available  on UART4, UART5. */
#define MAPLE_USART_CR2_STOP_BITS_1_POINT_5   (0x3 << 12)
/**
 * @brief Clock enable.
 * Not available on UART4, UART5 */
#define MAPLE_USART_CR2_CLKEN                 BIT(MAPLE_USART_CR2_CLKEN_BIT)
/**
 * @brief Clock polarity mask.
 * Not available on UART4, UART5 */
#define MAPLE_USART_CR2_CPOL                  BIT(MAPLE_USART_CR2_CPOL_BIT)
/** Clock polarity: low */
#define MAPLE_USART_CR2_CPOL_LOW              (0x0 << MAPLE_USART_CR2_CLKEN_BIT)
/** Clock polarity: high */
#define MAPLE_USART_CR2_CPOL_HIGH             (0x1 << MAPLE_USART_CR2_CLKEN_BIT)
/**
 * @brief Clock phase mask.
 * Not available on UART4, UART5 */
#define MAPLE_USART_CR2_CPHA                  BIT(MAPLE_USART_CR2_CPHA_BIT)
/**
 * @brief Clock phase: first
 * First clock transition is the first data capture edge. */
#define MAPLE_USART_CR2_CPHA_FIRST            (0x0 << MAPLE_USART_CR2_CPHA_BIT)
/**
 * @brief Clock phase: second
 * Second clock transition is the first data capture edge. */
#define MAPLE_USART_CR2_CPHA_SECOND           (0x1 << MAPLE_USART_CR2_CPHA_BIT)
/**
 * @brief Last bit clock pulse mask.
 *
 * When set, the last bit transmitted causes a clock pulse in
 * synchronous mode.
 *
 * Not available on UART4, UART5 */
#define MAPLE_USART_CR2_LBCL                  BIT(MAPLE_USART_CR2_LBCL_BIT)
/** LIN break detection interrupt enable mask. */
#define MAPLE_USART_CR2_LBDIE                 BIT(MAPLE_USART_CR2_LBDIE_BIT)
/** LIN break detection length. */
#define MAPLE_USART_CR2_LBDL                  BIT(MAPLE_USART_CR2_LBDL_BIT)
/** LIN break detection length: 10 bits */
#define MAPLE_USART_CR2_LBDL_10_BIT           (0 << MAPLE_USART_CR2_LBDL_BIT)
/** LIN break detection length: 11 bits */
#define MAPLE_USART_CR2_LBDL_11_BIT           (1 << MAPLE_USART_CR2_LBDL_BIT)
/**
 * @brief Address of the MAPLE_USART node
 * This is useful during multiprocessor communication. */
#define MAPLE_USART_CR2_ADD                   0xF

/* Control register 3 */

/** Clear to send interrupt enable bit */
#define MAPLE_USART_CR3_CTSIE_BIT             10
/** Clear to send enable bit */
#define MAPLE_USART_CR3_CTSE_BIT              9
/** Ready to send enable bit */
#define MAPLE_USART_CR3_RTSE_BIT              8
/** DMA enable transmitter bit */
#define MAPLE_USART_CR3_DMAT_BIT              7
/** DMA enable receiver bit */
#define MAPLE_USART_CR3_DMAR_BIT              6
/** Smartcard mode enable bit */
#define MAPLE_USART_CR3_SCEN_BIT              5
/** Smartcard NACK enable bit */
#define MAPLE_USART_CR3_NACK_BIT              4
/** Half-duplex selection bit */
#define MAPLE_USART_CR3_HDSEL_BIT             3
/** IrDA low power bit */
#define MAPLE_USART_CR3_IRLP_BIT              2
/** IrDA mode enable bit */
#define MAPLE_USART_CR3_IREN_BIT              1
/** Error interrupt enable bit */
#define MAPLE_USART_CR3_EIE_BIT               0

/**
 * @brief Clear to send interrupt enable
 * Not available on UART4, UART5. */
#define MAPLE_USART_CR3_CTSIE                 BIT(MAPLE_USART_CR3_CTSIE_BIT)
/**
 * @brief Clear to send enable
 * Not available on UART4, UART5. */
#define MAPLE_USART_CR3_CTSE                  BIT(MAPLE_USART_CR3_CTSE_BIT)
/**
 * @brief Ready to send enable
 * Not available on UART4, UART5. */
#define MAPLE_USART_CR3_RTSE                  BIT(MAPLE_USART_CR3_RTSE_BIT)
/**
 * @brief DMA enable transmitter
 * Not available on UART5. */
#define MAPLE_USART_CR3_DMAT                  BIT(MAPLE_USART_CR3_DMAT_BIT)
/**
 * @brief DMA enable receiver
 * Not available on UART5. */
#define MAPLE_USART_CR3_DMAR                  BIT(MAPLE_USART_CR3_DMAR_BIT)
/**
 * @brief Smartcard mode enable
 * Not available on UART4, UART5. */
#define MAPLE_USART_CR3_SCEN                  BIT(MAPLE_USART_CR3_SCEN_BIT)
/**
 * @brief Smartcard NACK enable
 * Not available on UART4, UART5. */
#define MAPLE_USART_CR3_NACK                  BIT(MAPLE_USART_CR3_NACK_BIT)
/**
 * @brief Half-duplex selection
 * When set, single-wire half duplex mode is selected.
 */
#define MAPLE_USART_CR3_HDSEL                 BIT(MAPLE_USART_CR3_HDSEL_BIT)
/** IrDA low power mode */
#define MAPLE_USART_CR3_IRLP                  BIT(MAPLE_USART_CR3_IRLP_BIT)
/** IrDA mode: normal */
#define MAPLE_USART_CR3_IRLP_NORMAL           (0U << MAPLE_USART_CR3_IRLP_BIT)
/** IrDA mode: low power */
#define MAPLE_USART_CR3_IRLP_LOW_POWER        (1U << MAPLE_USART_CR3_IRLP_BIT)
/** IrDA mode enable */
#define MAPLE_USART_CR3_IREN                  BIT(MAPLE_USART_CR3_IREN_BIT)
/** Error interrupt enable */
#define MAPLE_USART_CR3_EIE                   BIT(MAPLE_USART_CR3_EIE_BIT)

/* Guard time and prescaler register */

/**
 * @brief Guard time value mask
 * Used in Smartcard mode. Not available on UART4, UART5. */
#define MAPLE_USART_GTPR_GT                   (0xFF << 8)
/**
 * @brief Prescaler value mask
 * Restrictions on this value apply, depending on the MAPLE_USART mode. Not
 * available on UART4, UART5. */
#define MAPLE_USART_GTPR_PSC                  0xFF

/*
 * Devices
 */

#ifndef MAPLE_USART_RX_BUF_SIZE
#define MAPLE_USART_RX_BUF_SIZE               64
#endif

/** MAPLE_USART device type */
typedef struct usart_dev {
    usart_reg_map *regs;             /**< Register map */
    ring_buffer *rb;                 /**< RX ring buffer */
    uint32 max_baud;                 /**< @brief Deprecated.
                                      * Maximum baud rate. */
    uint8 rx_buf[MAPLE_USART_RX_BUF_SIZE]; /**< @brief Deprecated.
                                      * Actual RX buffer used by rb.
                                      * This field will be removed in
                                      * a future release. */
    rcc_clk_id clk_id;               /**< RCC clock information */
    nvic_irq_num irq_num;            /**< MAPLE_USART NVIC interrupt */
} usart_dev;

void usart_init(usart_dev *dev);

struct gpio_dev;                /* forward declaration */
/* FIXME [PRE 0.0.13] decide if flags are necessary */
/**
 * @brief Configure GPIOs for use as MAPLE_USART TX/RX.
 * @param udev MAPLE_USART device to use
 * @param rx_dev RX pin gpio_dev
 * @param rx     RX pin bit on rx_dev
 * @param tx_dev TX pin gpio_dev
 * @param tx     TX pin bit on tx_dev
 * @param flags  Currently ignored
 */
extern void usart_config_gpios_async(usart_dev *udev,
                                     struct gpio_dev *rx_dev, uint8 rx,
                                     struct gpio_dev *tx_dev, uint8 tx,
                                     unsigned flags);

#define MAPLE_USART_USE_PCLK 0
void usart_set_baud_rate(usart_dev *dev, uint32 clock_speed, uint32 baud);

void usart_enable(usart_dev *dev);
void usart_disable(usart_dev *dev);
void usart_foreach(void (*fn)(usart_dev *dev));
uint32 usart_tx(usart_dev *dev, const uint8 *buf, uint32 len);
uint32 usart_rx(usart_dev *dev, uint8 *buf, uint32 len);
void usart_putudec(usart_dev *dev, uint32 val);

/**
 * @brief Disable all serial ports.
 */
static inline void usart_disable_all(void) {
    usart_foreach(usart_disable);
}

/**
 * @brief Transmit one character on a serial port.
 *
 * This function blocks until the character has been successfully
 * transmitted.
 *
 * @param dev Serial port to send on.
 * @param byte Byte to transmit.
 */
static inline void usart_putc(usart_dev* dev, uint8 byte) {
    while (!usart_tx(dev, &byte, 1))
        ;
}

/**
 * @brief Transmit a character string on a serial port.
 *
 * This function blocks until str is completely transmitted.
 *
 * @param dev Serial port to send on
 * @param str String to send
 */
static inline void usart_putstr(usart_dev *dev, const char* str) {
    uint32 i = 0;
    while (str[i] != '\0') {
        usart_putc(dev, str[i++]);
    }
}

/**
 * @brief Read one character from a serial port.
 *
 * It's not safe to call this function if the serial port has no data
 * available.
 *
 * @param dev Serial port to read from
 * @return byte read
 * @see usart_data_available()
 */
static inline uint8 usart_getc(usart_dev *dev) {
    return rb_remove(dev->rb);
}

/*
 * Roger Clark. 20141125, 
 * added peek function.
 * @param dev Serial port to read from
 * @return byte read
 */
static inline int usart_peek(usart_dev *dev)
{
	return rb_peek(dev->rb);
}


/**
 * @brief Return the amount of data available in a serial port's RX buffer.
 * @param dev Serial port to check
 * @return Number of bytes in dev's RX buffer.
 */
static inline uint32 usart_data_available(usart_dev *dev) {
    return rb_full_count(dev->rb);
}

/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_rx(usart_dev *dev) {
    rb_reset(dev->rb);
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
