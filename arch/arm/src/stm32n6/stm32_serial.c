/****************************************************************************
 * arch/arm/src/stm32n6/stm32_serial.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_uart.h"

#include "stm32_rcc.h"
#include "arm_internal.h"

#if defined(CONFIG_SERIAL_TXDMA) && defined(CONFIG_STM32_GPDMA1) && \
    defined(CONFIG_USART1_TXDMA)
#  define STM32_USART1_TXDMA
#endif

#ifdef STM32_USART1_TXDMA
#  include "stm32_dma.h"
#  if defined(CONFIG_STM32_GPDMA1) && defined(CONFIG_USART1_TXDMA)
#    include "hardware/stm32n6xxx_dmasigmap.h"
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_STM32_PM_SERIAL_ACTIVITY)
#  define CONFIG_STM32_PM_SERIAL_ACTIVITY  10
#endif

/* USART Unconfigure bits */

#define USART_UNCONFIGURE_RX                    (1 << 0)
#define USART_UNCONFIGURE_TX                    (1 << 1)

/* Keep track if a Break was set
 *
 * Note:
 *
 * 1) This value is set in the priv->ie but never written to the control
 *    register. It must not collide with USART_CR1_USED_INTS or USART_CR3_EIE
 * 2) USART_CR3_EIE is also carried in the up_dev_s ie member.
 *
 * See stm32serial_restoreusartint where the masking is done.
 */

#ifdef CONFIG_STM32_SERIALBRK_BSDCOMPAT
#  define USART_CR1_IE_BREAK_INPROGRESS_SHFTS 15
#  define USART_CR1_IE_BREAK_INPROGRESS (1 << USART_CR1_IE_BREAK_INPROGRESS_SHFTS)
#endif

#ifdef USE_SERIALDRIVER
#ifdef HAVE_UART

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_serial_s
{
  struct uart_dev_s dev;       /* Generic UART device */
  uint16_t          ie;        /* Saved interrupt mask bits value */

  /* Has been initialized and HW is setup. */

  bool              initialized;
  int               shutdown_error;

  /* If termios are supported, then the following fields may vary at
   * runtime.
   */

#ifdef CONFIG_SERIAL_TERMIOS
  uint8_t           parity;    /* 0=none, 1=odd, 2=even */
  uint8_t           bits;      /* Number of bits (7 or 8) */
  bool              stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  bool              oflow;     /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  bool              iflow;     /* input flow control (RTS) enabled */
#endif
  uint32_t          baud;      /* Configured baud */
#else
  const uint8_t     parity;    /* 0=none, 1=odd, 2=even */
  const uint8_t     bits;      /* Number of bits (7 or 8) */
  const bool        stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const bool        oflow;     /* output flow control (CTS) enabled */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const bool        iflow;     /* input flow control (RTS) enabled */
#endif
  const uint32_t    baud;      /* Configured baud */
#endif
  const uint8_t     irq;       /* IRQ associated with this USART */
  const uint32_t    usartbase; /* Base address of USART registers */
  const uint32_t    tx_gpio;   /* U[S]ART TX GPIO pin configuration */
  const uint32_t    rx_gpio;   /* U[S]ART RX GPIO pin configuration */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  const uint32_t    rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  const uint32_t    cts_gpio;  /* U[S]ART CTS GPIO pin configuration */
#endif
  const uint8_t     unconfigure; /* Unconfigure pins on close */
  spinlock_t        lock;

#ifdef STM32_USART1_TXDMA
  DMA_HANDLE        txdma;
  uintptr_t         txdma_buffer;
  size_t            txdma_length;
  volatile bool     txdma_active;
  volatile bool     txdma_fallback;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  stm32serial_setup(struct uart_dev_s *dev);
static void stm32serial_shutdown(struct uart_dev_s *dev);
static int  stm32serial_attach(struct uart_dev_s *dev);
static void stm32serial_detach(struct uart_dev_s *dev);
static int  stm32serial_interrupt(int irq, void *context, void *arg);
static int  stm32serial_ioctl(struct file *filep,
                                int cmd, unsigned long arg);
static int  stm32serial_receive(struct uart_dev_s *dev,
                                unsigned int *status);
static void stm32serial_rxint(struct uart_dev_s *dev, bool enable);
static bool stm32serial_rxavailable(struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool stm32serial_rxflowcontrol(struct uart_dev_s *dev,
                                        unsigned int nbuffered, bool upper);
#endif
static void stm32serial_send(struct uart_dev_s *dev, int ch);
static void stm32serial_txint(struct uart_dev_s *dev, bool enable);
static bool stm32serial_txready(struct uart_dev_s *dev);
static bool stm32serial_txempty(struct uart_dev_s *dev);

#ifdef STM32_USART1_TXDMA
static void stm32serial_dmainitialize(struct stm32_serial_s *priv);
static void stm32serial_dmasend(struct uart_dev_s *dev);
static void stm32serial_dmatxavail(struct uart_dev_s *dev);
static void stm32serial_dmatxcallback(DMA_HANDLE handle, uint8_t status,
                                      void *arg);
static void stm32serial_dmafallback(struct stm32_serial_s *priv, int error);
#endif

#ifdef CONFIG_PM
static int  stm32serial_pmprepare(struct pm_callback_s *cb, int domain,
                                    enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = stm32serial_setup,
  .shutdown       = stm32serial_shutdown,
  .attach         = stm32serial_attach,
  .detach         = stm32serial_detach,
  .ioctl          = stm32serial_ioctl,
  .receive        = stm32serial_receive,
  .rxint          = stm32serial_rxint,
  .rxavailable    = stm32serial_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = stm32serial_rxflowcontrol,
#endif
#ifdef STM32_USART1_TXDMA
  .dmasend        = stm32serial_dmasend,
  .dmatxavail     = stm32serial_dmatxavail,
#endif
  .send           = stm32serial_send,
  .txint          = stm32serial_txint,
  .txready        = stm32serial_txready,
  .txempty        = stm32serial_txempty,
};

/* I/O buffers */

#ifdef CONFIG_STM32_USART1_SERIALDRIVER
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif

/* This describes the state of the STM32N6 USART1 port. */

#ifdef CONFIG_STM32_USART1_SERIALDRIVER
static struct stm32_serial_s g_usart1priv =
{
  .dev =
    {
#  if CONSOLE_UART == 1
      .isconsole = true,
#  endif
      .recv      =
      {
        .size    = CONFIG_USART1_RXBUFSIZE,
        .buffer  = g_usart1rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART1_TXBUFSIZE,
        .buffer  = g_usart1txbuffer,
      },
      .ops       = &g_uart_ops,
      .priv      = &g_usart1priv,
    },

  .irq           = STM32_IRQ_USART1,
  .parity        = CONFIG_USART1_PARITY,
  .bits          = CONFIG_USART1_BITS,
  .stopbits2     = CONFIG_USART1_2STOP,
  .baud          = CONFIG_USART1_BAUD,
  .usartbase     = STM32_USART1_BASE,
  .tx_gpio       = GPIO_USART1_TX,
  .rx_gpio       = GPIO_USART1_RX,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART1_OFLOWCONTROL)
  .oflow         = true,
  .cts_gpio      = GPIO_USART1_CTS,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART1_IFLOWCONTROL)
  .iflow         = true,
  .rts_gpio      = GPIO_USART1_RTS,
#  endif

  .lock               = SP_UNLOCKED,
  .unconfigure        = 0
#if defined(CONFIG_USART1_UNCONFIG_RX_ON_CLOSE)
                      |
                      USART_UNCONFIGURE_RX
#endif
#if defined(CONFIG_USART1_UNCONFIG_TX_ON_CLOSE)
                      |
                      USART_UNCONFIGURE_TX
#endif
      ,
};
#endif

/* This table lets us iterate over the configured USARTs */

static struct stm32_serial_s * const
  g_uart_devs[STM32_NUSART] =
{
#ifdef CONFIG_STM32_USART1_SERIALDRIVER
  [0] = &g_usart1priv,
#endif
};

#ifdef CONFIG_PM
static struct pm_callback_s g_serialpm =
{
  .prepare = stm32serial_pmprepare
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32serial_getreg
 ****************************************************************************/

static inline
uint32_t stm32serial_getreg(struct stm32_serial_s *priv, int offset)
{
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: stm32serial_putreg
 ****************************************************************************/

static inline
void stm32serial_putreg(struct stm32_serial_s *priv,
                          int offset, uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: stm32serial_setusartint
 ****************************************************************************/

static inline
void stm32serial_setusartint(struct stm32_serial_s *priv,
                               uint16_t ie)
{
  uint32_t cr;

  /* Save the interrupt mask */

  priv->ie = ie;

  /* And restore the interrupt state (see the interrupt enable/usage table
   * above)
   */

  cr = stm32serial_getreg(priv, STM32_USART_CR1_OFFSET);
  cr &= ~(USART_CR1_USED_INTS);
  cr |= (ie & (USART_CR1_USED_INTS));
  stm32serial_putreg(priv, STM32_USART_CR1_OFFSET, cr);

  cr = stm32serial_getreg(priv, STM32_USART_CR3_OFFSET);
  cr &= ~USART_CR3_EIE;
  cr |= (ie & USART_CR3_EIE);
  stm32serial_putreg(priv, STM32_USART_CR3_OFFSET, cr);
}

/****************************************************************************
 * Name: stm32serial_restoreusartint
 ****************************************************************************/

static void stm32serial_restoreusartint(struct stm32_serial_s *priv,
                                          uint16_t ie)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  stm32serial_setusartint(priv, ie);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: stm32serial_disableusartint
 ****************************************************************************/

static void stm32serial_disableusartint(struct stm32_serial_s *priv,
                                          uint16_t *ie)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (ie)
    {
      uint32_t cr1;
      uint32_t cr3;

      /* USART interrupts:
       *
       * Enable           Status         Meaning                Usage
       * ---------------- -------------- ---------------------- ----------
       * USART_CR1_IDLEIE USART_ISR_IDLE Idle Line Detected     (not used)
       * USART_CR1_RXNEIE USART_ISR_RXNE Received Data Ready
       *                                 to be Read
       * "              " USART_ISR_ORE  Overrun Error Detected
       * USART_CR1_TCIE   USART_ISR_TC   Transmission Complete  (RS-485)
       * USART_CR1_TXEIE  USART_ISR_TXE  Transmit Data Register
       *                                 Empty
       * USART_CR1_PEIE   USART_ISR_PE   Parity Error
       *
       * USART_CR2_LBDIE  USART_ISR_LBD  Break Flag             (not used)
       * USART_CR3_EIE    USART_ISR_FE   Framing Error
       * "           "    USART_ISR_NF   Noise Flag
       * "           "    USART_ISR_ORE  Overrun Error Detected
       * USART_CR3_CTSIE  USART_ISR_CTS  CTS flag               (not used)
       */

      cr1 = stm32serial_getreg(priv, STM32_USART_CR1_OFFSET);
      cr3 = stm32serial_getreg(priv, STM32_USART_CR3_OFFSET);

      /* Return the current interrupt mask value for the used interrupts.
       * Notice that this depends on the fact that none of the used interrupt
       * enable bits overlap.  This logic would fail if we needed the break
       * interrupt!
       */

      *ie = (cr1 & (USART_CR1_USED_INTS)) | (cr3 & USART_CR3_EIE);
    }

  /* Disable all interrupts */

  stm32serial_setusartint(priv, 0);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: stm32serial_busy
 *
 * Description:
 *   Called with IRQs excluded before changing an active peripheral.
 *
 ****************************************************************************/

#if defined(CONFIG_PM) || (defined(CONFIG_SERIAL_TERMIOS) && \
                          !defined(CONFIG_SUPPRESS_UART_CONFIG))
static bool stm32serial_busy(struct stm32_serial_s *priv)
{
  uint32_t sr;

#ifdef STM32_USART1_TXDMA
  if (priv->txdma_active)
    {
      return true;
    }
#endif

  sr = stm32serial_getreg(priv, STM32_USART_ISR_OFFSET);
  return priv->dev.xmit.head != priv->dev.xmit.tail ||
         (sr & USART_ISR_TC) == 0 ||
         (sr & (USART_ISR_RXNE | USART_ISR_BUSY)) != 0 ||
         (stm32serial_getreg(priv, STM32_USART_CR3_OFFSET) &
          (USART_CR3_DMAT | USART_CR3_DMAR)) != 0;
}
#endif

/****************************************************************************
 * Name: stm32serial_setapbclock
 *
 * Description:
 *   Enable or disable APB clock for the USART peripheral
 *
 * Input Parameters:
 *   dev - A reference to the UART driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void stm32serial_setapbclock(struct uart_dev_s *dev, bool on)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  uint32_t rcc_en;
  uint32_t regaddr_set;
  uint32_t regaddr_clr;

  /* Determine which USART to configure */

  switch (priv->usartbase)
    {
    default:
      return;
#ifdef CONFIG_STM32_USART1_SERIALDRIVER
    case STM32_USART1_BASE:
      rcc_en = RCC_APB2ENR_USART1EN;
      regaddr_set = STM32_RCC_APB2ENSR;
      regaddr_clr = STM32_RCC_APB2ENCR;
      break;
#endif
    }

  /* Enable/disable APB 1/2 clock for USART.
   * Use atomic SET/CLEAR registers (ENSR/ENCR) instead of read-modify-write
   * on ENR because the RCC's internal RIF security may block ENR access.
   */

  if (on)
    {
      putreg32(rcc_en, regaddr_set);
    }
  else
    {
      putreg32(rcc_en, regaddr_clr);
    }
}

/****************************************************************************
 * Name: stm32serial_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int stm32serial_setup(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  irqstate_t flags;
  int ret = OK;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct stm32_usart_format_s format;
  uint32_t flow = 0;
#endif

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->initialized)
    {
      ret = priv->shutdown_error;
      goto out;
    }

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  ret = stm32_usart_format(stm32_usart_clock(), priv->baud, priv->bits,
                           priv->parity, priv->stopbits2, &format);
  if (ret < 0)
    {
      goto out;
    }

  stm32serial_setapbclock(dev, true);

  if (priv->tx_gpio != 0)
    {
      ret = stm32_configgpio(priv->tx_gpio);
      if (ret < 0)
        {
          goto out;
        }
    }

  if (priv->rx_gpio != 0)
    {
      ret = stm32_configgpio(priv->rx_gpio);
      if (ret < 0)
        {
          goto out;
        }
    }

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      ret = stm32_configgpio(priv->cts_gpio);
      if (ret < 0)
        {
          goto out;
        }

      if (priv->oflow)
        {
          flow |= USART_CR3_CTSE;
        }
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      uint32_t config = priv->rts_gpio;

#ifdef CONFIG_STM32_FLOWCONTROL_BROKEN
      /* Instead of letting hw manage this pin, we will bitbang */

      config = (config & ~GPIO_MODE_MASK) | GPIO_OUTPUT;
#endif
      ret = stm32_configgpio(config);
      if (ret < 0)
        {
          goto out;
        }

#ifndef CONFIG_STM32_FLOWCONTROL_BROKEN
      if (priv->iflow)
        {
          flow |= USART_CR3_RTSE;
        }
#endif
    }
#endif

  ret = stm32_usart_configure(priv->usartbase, &format, flow);
  if (ret < 0)
    {
      goto out;
    }

#endif /* CONFIG_SUPPRESS_UART_CONFIG */

  /* Set up the cached interrupt enables value */

  priv->ie    = 0;

  /* Mark device as initialized. */

  priv->initialized = true;
  priv->shutdown_error = OK;

out:
  spin_unlock_irqrestore(&priv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: stm32serial_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void stm32serial_shutdown(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  int ret;

  /* Disable all interrupts */

  stm32serial_disableusartint(priv, NULL);

#ifdef STM32_USART1_TXDMA
  if (priv->txdma != NULL)
    {
      uint32_t cr3;
      cr3 = stm32serial_getreg(priv, STM32_USART_CR3_OFFSET);
      stm32serial_putreg(priv, STM32_USART_CR3_OFFSET,
                         cr3 & ~USART_CR3_DMAT);

      ret = stm32_dmastop(priv->txdma);
      if (ret < 0)
        {
          _err("ERROR: USART TX DMA stop failed: %d\n", ret);
          priv->shutdown_error = ret;
          return;
        }

      priv->txdma_active = false;

      ret = stm32_dmafree(priv->txdma);
      if (ret < 0)
        {
          _err("ERROR: USART TX DMA release failed: %d\n", ret);
        }
      else
        {
          priv->txdma = NULL;
        }
    }
#endif

  /* A nonblocking close or upper-half drain timeout may discard wire TX. */

  if ((stm32serial_getreg(priv, STM32_USART_ISR_OFFSET) & USART_ISR_TC) == 0)
    {
      _warn("WARNING: USART close discarding unfinished wire TX\n");
    }

  ret = stm32_usart_disable(priv->usartbase);
  if (ret < 0)
    {
      _err("ERROR: USART disable failed: %d\n", ret);
      priv->shutdown_error = ret;
      return;
    }

  /* Disable the peripheral before removing its register bus clock. */

  stm32serial_setapbclock(dev, false);
  priv->initialized = false;
  priv->shutdown_error = OK;

  /* Release pins. "If the serial-attached device is powered down, the TX
   * pin causes back-powering, potentially confusing the device to the point
   * of complete lock-up."
   *
   * REVISIT:  Is unconfiguring the pins appropriate for all devices?  If
   * not, then this may need to be a configuration option.
   */

  if ((priv->unconfigure & USART_UNCONFIGURE_TX) && (priv->tx_gpio != 0))
    {
      stm32_unconfiggpio(priv->tx_gpio);
    }

  if ((priv->unconfigure & USART_UNCONFIGURE_RX) && (priv->rx_gpio != 0))
    {
      stm32_unconfiggpio(priv->rx_gpio);
    }

#ifdef CONFIG_SERIAL_OFLOWCONTROL
  if (priv->cts_gpio != 0)
    {
      stm32_unconfiggpio(priv->cts_gpio);
    }
#endif

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  if (priv->rts_gpio != 0)
    {
      stm32_unconfiggpio(priv->rts_gpio);
    }
#endif
}

/****************************************************************************
 * Name: stm32serial_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int stm32serial_attach(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  int ret;

#ifdef STM32_USART1_TXDMA
  /* Early serial setup runs before arm_dma_initialize(). */

  if (priv->txdma == NULL)
    {
      stm32serial_dmainitialize(priv);
    }
#endif

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, stm32serial_interrupt, priv);

  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the USART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32serial_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void stm32serial_detach(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: stm32serial_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int stm32serial_interrupt(int irq, void *context, void *arg)
{
  struct stm32_serial_s *priv = (struct stm32_serial_s *)arg;
  int  passes;
  bool handled;
  uint32_t sr;

  DEBUGASSERT(priv != NULL);

  /* Report serial activity to the power management logic */

#if defined(CONFIG_PM) && CONFIG_STM32_PM_SERIAL_ACTIVITY > 0
  pm_activity(PM_IDLE_DOMAIN, CONFIG_STM32_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked USART status word. */

      sr = stm32serial_getreg(priv, STM32_USART_ISR_OFFSET);

      /* USART interrupts:
       *
       * Enable           Status         Meaning                Usage
       * ---------------- -------------- ---------------------- ----------
       * USART_CR1_IDLEIE USART_ISR_IDLE Idle Line Detected     (not used)
       * USART_CR1_RXNEIE USART_ISR_RXNE Received Data Ready to
       *                                 be Read
       * "              " USART_ISR_ORE  Overrun Error Detected
       * USART_CR1_TCIE   USART_ISR_TC   Transmission Complete  (RS-485)
       * USART_CR1_TXEIE  USART_ISR_TXE  Transmit Data Register
       *                                 Empty
       * USART_CR1_PEIE   USART_ISR_PE   Parity Error
       *
       * USART_CR2_LBDIE  USART_ISR_LBD  Break Flag             (not used)
       * USART_CR3_EIE    USART_ISR_FE   Framing Error
       * "           "    USART_ISR_NE   Noise Error
       * "           "    USART_ISR_ORE  Overrun Error Detected
       * USART_CR3_CTSIE  USART_ISR_CTS  CTS flag               (not used)
       *
       * NOTE: Some of these status bits must be cleared by explicitly
       * writing one to the ICR register: USART_ICR_CTSCF, USART_ICR_LBDCF.
       * None of those are currently being used.
       */

      /* Handle incoming, receive bytes. */

      if ((sr & USART_ISR_RXNE) != 0 &&
          (priv->ie & USART_CR1_RXNEIE) != 0)
        {
          /* Received data ready... process incoming bytes.  NOTE the check
           * for RXNEIE:  We cannot call uart_recvchars if RX interrupts are
           * disabled.
           */

          uart_recvchars(&priv->dev);
          handled = true;
        }

      /* We may still have to read from the DR register to clear any pending
       * error conditions.
       */

      else if ((sr & USART_ISR_RXNE) == 0 &&
               (sr & USART_ISR_ERRORS) != 0)
        {
          /* These errors are cleared by writing the corresponding bit to the
           * interrupt clear register (ICR).
           */

          stm32serial_putreg(priv, STM32_USART_ICR_OFFSET,
                             sr & USART_ISR_ERRORS);
          handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((sr & USART_ISR_TXE) != 0 &&
          (priv->ie & USART_CR1_TXEIE) != 0)
        {
          /* Transmit data register empty ... process outgoing bytes */

          uart_xmitchars(&priv->dev);
          handled = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32serial_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int stm32serial_ioctl(struct file *filep, int cmd,
                               unsigned long arg)
{
  struct inode          *inode = filep->f_inode;
  struct uart_dev_s     *dev   = inode->i_private;
  struct stm32_serial_s *priv  = (struct stm32_serial_s *)dev->priv;
  int                    ret   = OK;

  UNUSED(inode);
  UNUSED(dev);
  UNUSED(priv);

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct stm32_serial_s *user;

        user = (struct stm32_serial_s *)arg;

        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct stm32_serial_s));
          }
      }
      break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        irqstate_t flags;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        flags = spin_lock_irqsave(&priv->lock);
        cfsetispeed(termiosp, priv->baud);
        cfsetospeed(termiosp, priv->baud);

        termiosp->c_cflag =
          ((priv->parity != 0) ? PARENB : 0) |
          ((priv->parity == 1) ? PARODD : 0) |
          ((priv->stopbits2) ? CSTOPB : 0) |
#ifdef CONFIG_SERIAL_OFLOWCONTROL
          ((priv->oflow) ? CCTS_OFLOW : 0) |
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
          ((priv->iflow) ? CRTS_IFLOW : 0) |
#endif
          (priv->bits == 7 ? CS7 : CS8);
        spin_unlock_irqrestore(&priv->lock, flags);
      }
      break;

    case TCSETS:
      {
#ifdef CONFIG_SUPPRESS_UART_CONFIG
        ret = -ENOTSUP;
#else
        struct termios *termiosp = (struct termios *)arg;
        struct stm32_usart_format_s format;
        irqstate_t flags;
        uint32_t baud;
        uint32_t flow = 0;
        uint8_t bits;
        uint8_t parity;
        uint16_t ie;
        bool stopbits2;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Perform some sanity checks before accepting any changes */

        if (((termiosp->c_cflag & CSIZE) != CS7 &&
             (termiosp->c_cflag & CSIZE) != CS8)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && (priv->cts_gpio == 0))
#else
            || (termiosp->c_cflag & CCTS_OFLOW) != 0
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && (priv->rts_gpio == 0))
#else
            || (termiosp->c_cflag & CRTS_IFLOW) != 0
#endif
           )
          {
            ret = -EINVAL;
            break;
          }

        bits = (termiosp->c_cflag & CSIZE) == CS7 ? 7 : 8;
        parity = (termiosp->c_cflag & PARENB) == 0 ? 0 :
                 (termiosp->c_cflag & PARODD) != 0 ? 1 : 2;
        stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
        baud = cfgetispeed(termiosp);

        flags = spin_lock_irqsave(&priv->lock);
        ret = stm32_usart_format(stm32_usart_clock(), baud, bits, parity,
                                 stopbits2, &format);
        if (ret < 0)
          {
            goto format_out;
          }

        if (!priv->initialized || stm32serial_busy(priv))
          {
            ret = -EBUSY;
            goto format_out;
          }

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        if ((termiosp->c_cflag & CCTS_OFLOW) != 0)
          {
            flow |= USART_CR3_CTSE;
          }

#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && !defined(CONFIG_STM32_FLOWCONTROL_BROKEN)
        if ((termiosp->c_cflag & CRTS_IFLOW) != 0)
          {
            flow |= USART_CR3_RTSE;
          }
#endif

        /* The upper half implements drain/flush.  Preserve its RX ring;
         * frames arriving at the UE-disable boundary cannot be preserved.
         */

        ie = priv->ie;
        stm32serial_setusartint(priv, 0);
        ret = stm32_usart_configure(priv->usartbase, &format, flow);
        if (ret == OK)
          {
            priv->baud = baud;
            priv->bits = bits;
            priv->parity = parity;
            priv->stopbits2 = stopbits2;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            priv->oflow = (termiosp->c_cflag & CCTS_OFLOW) != 0;
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            priv->iflow = (termiosp->c_cflag & CRTS_IFLOW) != 0;
#endif
          }

        stm32serial_setusartint(priv, ie);

format_out:
        spin_unlock_irqrestore(&priv->lock, flags);
#endif
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: stm32serial_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int stm32serial_receive(struct uart_dev_s *dev,
                                 unsigned int *status)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  uint32_t rdr;
  uint32_t sr;

  /* FIFO errors belong to the head character.  Clear before popping RDR,
   * otherwise an ICR write could clear the following character's errors.
   */

  sr = stm32serial_getreg(priv, STM32_USART_ISR_OFFSET);
  if ((sr & USART_ISR_ERRORS) != 0)
    {
      stm32serial_putreg(priv, STM32_USART_ICR_OFFSET,
                         sr & USART_ISR_ERRORS);
    }

  rdr = stm32serial_getreg(priv, STM32_USART_RDR_OFFSET);
  *status = (sr & 0xffff) << 16 | rdr;

  return rdr & (priv->bits == 7 ? 0x7f : 0xff);
}

/****************************************************************************
 * Name: stm32serial_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void stm32serial_rxint(struct uart_dev_s *dev, bool enable)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  irqstate_t flags;
  uint16_t ie;

  /* USART receive interrupts:
   *
   * Enable           Status         Meaning                  Usage
   * ---------------- -------------- ---------------------- ----------
   * USART_CR1_IDLEIE USART_ISR_IDLE Idle Line Detected     (not used)
   * USART_CR1_RXNEIE USART_ISR_RXNE Received Data Ready
   *                                    to be Read
   * "              " USART_ISR_ORE  Overrun Error Detected
   * USART_CR1_PEIE   USART_ISR_PE   Parity Error
   *
   * USART_CR2_LBDIE  USART_ISR_LBD  Break Flag              (not used)
   * USART_CR3_EIE    USART_ISR_FE   Framing Error
   * "           "    USART_ISR_NF   Noise Flag
   * "           "    USART_ISR_ORE  Overrun Error Detected
   */

  flags = enter_critical_section();
  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data
       * register (or an Rx timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#ifdef CONFIG_USART_ERRINTS
      ie |= (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR3_EIE);
#else
      ie |= USART_CR1_RXNEIE;
#endif
#endif
    }
  else
    {
      ie &= ~(USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR3_EIE);
    }

  /* Then set the new interrupt state */

  stm32serial_restoreusartint(priv, ie);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32serial_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool stm32serial_rxavailable(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;

  return ((stm32serial_getreg(priv, STM32_USART_ISR_OFFSET) &
           USART_ISR_RXNE) != 0);
}

/****************************************************************************
 * Name: stm32serial_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool stm32serial_rxflowcontrol(struct uart_dev_s *dev,
                                        unsigned int nbuffered, bool upper)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS) && \
    defined(CONFIG_STM32_FLOWCONTROL_BROKEN)
  if (priv->iflow && (priv->rts_gpio != 0))
    {
      /* Assert/de-assert nRTS set it high resume/stop sending */

      stm32_gpiowrite(priv->rts_gpio, upper);

      if (upper)
        {
          /* With heavy Rx traffic, RXNE might be set and data pending.
           * Returning 'true' in such case would cause RXNE left unhandled
           * and causing interrupt storm. Sending end might be also be slow
           * to react on nRTS, and returning 'true' here would prevent
           * processing that data.
           *
           * Therefore, return 'false' so input data is still being processed
           * until sending end reacts on nRTS signal and stops sending more.
           */

          return false;
        }

      return upper;
    }

#else
  if (priv->iflow)
    {
      /* Is the RX buffer full? */

      if (upper)
        {
          /* Disable Rx interrupt to prevent more data being from
           * peripheral.  When hardware RTS is enabled, this will
           * prevent more data from coming in.
           *
           * This function is only called when UART recv buffer is full,
           * that is: "dev->recv.head + 1 == dev->recv.tail".
           *
           * Logic in "uart_read" will automatically toggle Rx interrupts
           * when buffer is read empty and thus we do not have to re-
           * enable Rx interrupts.
           */

          uart_disablerxint(dev);
          return true;
        }

      /* No.. The RX buffer is empty */

      else
        {
          /* We might leave Rx interrupt disabled if full recv buffer was
           * read empty.  Enable Rx interrupt to make sure that more input is
           * received.
           */

          uart_enablerxint(dev);
        }
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: stm32serial_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void stm32serial_send(struct uart_dev_s *dev, int ch)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;

  stm32serial_putreg(priv, STM32_USART_TDR_OFFSET,
                     (uint32_t)ch & (priv->bits == 7 ? 0x7f : 0xff));
}

#ifdef STM32_USART1_TXDMA
/****************************************************************************
 * Name: stm32serial_dmainitialize
 *
 * Description:
 *   Allocate the GPDMA1 USART1 TX request. If unavailable, retain the
 *   interrupt-driven transmit path.
 *
 ****************************************************************************/

static void stm32serial_dmainitialize(struct stm32_serial_s *priv)
{
  priv->txdma = NULL;
  priv->txdma_active = false;
  priv->txdma_fallback = true;

#if defined(CONFIG_STM32_GPDMA1) && defined(CONFIG_USART1_TXDMA)
  {
    struct stm32_dma_request_s request =
    {
      .controller = STM32_DMA_CONTROLLER_GPDMA1,
      .direction = STM32_DMA_MEMORY_TO_PERIPHERAL,
      .request = STM32_DMA_REQ_USART1_TX,
      .peripheral_address = priv->usartbase + STM32_USART_TDR_OFFSET
    };
    int ret;

    priv->txdma = stm32_dmachannel(&request);
    if (priv->txdma == NULL)
      {
        _warn("WARNING: USART TX DMA channel unavailable; using interrupts\n");
        return;
      }

    ret = stm32_dmacallback(priv->txdma, stm32serial_dmatxcallback, priv);
    if (ret < 0)
      {
        _err("ERROR: USART TX DMA callback setup failed: %d\n", ret);
        stm32_dmafree(priv->txdma);
        priv->txdma = NULL;
        return;
      }

    priv->txdma_fallback = false;
  }
#endif
}

/****************************************************************************
 * Name: stm32serial_dmasend
 *
 * Description:
 *   Program one contiguous USART TX buffer segment and start DMA.
 *
 ****************************************************************************/

static void stm32serial_dmasend(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv = (struct stm32_serial_s *)dev->priv;
  struct stm32_dma_config_s config;
  uint32_t cr3;
  int ret;

  if (priv->txdma == NULL || priv->txdma_fallback)
    {
      stm32serial_dmafallback(priv, -ENODEV);
      return;
    }

  if (priv->txdma_length == 0)
    {
      priv->txdma_buffer = (uintptr_t)dev->dmatx.buffer;
      priv->txdma_length = dev->dmatx.length;
    }

  config.source_address = priv->txdma_buffer;
  config.destination_address =
    priv->usartbase + STM32_USART_TDR_OFFSET;
  config.nbytes = priv->txdma_length;
  config.width = 1;
  config.priority = 1;
  config.source_increment = true;
  config.destination_increment = false;

  ret = stm32_dmasetup(priv->txdma, &config);
  if (ret < 0)
    {
      stm32serial_dmafallback(priv, ret);
      return;
    }

  cr3 = stm32serial_getreg(priv, STM32_USART_CR3_OFFSET);
  stm32serial_putreg(priv, STM32_USART_CR3_OFFSET, cr3 | USART_CR3_DMAT);

  ret = stm32_dmastart(priv->txdma);
  if (ret < 0)
    {
      stm32serial_dmafallback(priv, ret);
    }
}

/****************************************************************************
 * Name: stm32serial_dmatxavail
 *
 * Description:
 *   Start DMA when queued TX data is available, or use UART interrupts when
 *   DMA is not available.
 *
 ****************************************************************************/

static void stm32serial_dmatxavail(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv = (struct stm32_serial_s *)dev->priv;
  irqstate_t flags;

  if (priv->txdma == NULL || priv->txdma_fallback)
    {
      stm32serial_txint(dev, true);
      return;
    }

  flags = enter_critical_section();
  if (!priv->txdma_active && dev->xmit.head != dev->xmit.tail)
    {
      priv->txdma_active = true;
      priv->txdma_buffer = 0;
      priv->txdma_length = 0;
      dev->dmatx.nbytes = 0;
      uart_xmitchars_dma(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32serial_dmafallback
 *
 * Description:
 *   Disable the failed DMA path and retry queued bytes using UART
 *   interrupts. Resetting nbytes keeps the TX buffer intact for retry.
 *
 ****************************************************************************/

static void stm32serial_dmafallback(struct stm32_serial_s *priv, int error)
{
  uint32_t cr3 = stm32serial_getreg(priv, STM32_USART_CR3_OFFSET);
  int ret;

  stm32serial_putreg(priv, STM32_USART_CR3_OFFSET, cr3 & ~USART_CR3_DMAT);

  if (priv->txdma != NULL)
    {
      ret = stm32_dmastop(priv->txdma);
      if (ret < 0)
        {
          _err("ERROR: USART TX DMA abort failed: %d\n", ret);
          return;
        }
    }

  priv->txdma_active = false;
  priv->txdma_fallback = true;
  priv->txdma_buffer = 0;
  priv->txdma_length = 0;
  priv->dev.dmatx.nbytes = 0;
  _err("ERROR: USART TX DMA failed (%d); using interrupts\n", error);
  uart_xmitchars_done(&priv->dev);
  stm32serial_txint(&priv->dev, true);
}

/****************************************************************************
 * Name: stm32serial_dmatxcallback
 *
 * Description:
 *   Account for a completed segment, submit a wrapped segment if present,
 *   then release the UART TX buffer for reuse.
 *
 ****************************************************************************/

static void stm32serial_dmatxcallback(DMA_HANDLE handle, uint8_t status,
                                      void *arg)
{
  struct stm32_serial_s *priv = (struct stm32_serial_s *)arg;
  uint32_t cr3;

  UNUSED(handle);

  if ((status & (DMA_STATUS_FATAL | DMA_STATUS_TOF | DMA_STATUS_SUSPF)) != 0)
    {
      stm32serial_dmafallback(priv, -EIO);
      return;
    }

  if ((status & DMA_STATUS_TCF) == 0)
    {
      return;
    }

  priv->dev.dmatx.nbytes += priv->txdma_length;

  if (priv->dev.dmatx.nlength > 0)
    {
      priv->txdma_buffer = (uintptr_t)priv->dev.dmatx.nbuffer;
      priv->txdma_length = priv->dev.dmatx.nlength;
      priv->dev.dmatx.length = priv->dev.dmatx.nlength;
      priv->dev.dmatx.nlength = 0;
      stm32serial_dmasend(&priv->dev);
      return;
    }

  cr3 = stm32serial_getreg(priv, STM32_USART_CR3_OFFSET);
  stm32serial_putreg(priv, STM32_USART_CR3_OFFSET, cr3 & ~USART_CR3_DMAT);
  priv->txdma_active = false;
  priv->txdma_buffer = 0;
  priv->txdma_length = 0;
  uart_xmitchars_done(&priv->dev);
  uart_dmatxavail(&priv->dev);
}
#endif

/****************************************************************************
 * Name: stm32serial_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void stm32serial_txint(struct uart_dev_s *dev, bool enable)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;
  irqstate_t flags;

#ifdef STM32_USART1_TXDMA
  if (priv->txdma != NULL && !priv->txdma_fallback)
    {
      if (enable)
        {
          stm32serial_dmatxavail(dev);
        }

      return;
    }
#endif

  /* USART transmit interrupts:
   *
   * Enable          Status        Meaning                      Usage
   * --------------- ------------- ---------------------------- ----------
   * USART_CR1_TCIE  USART_ISR_TC  Transmission Complete        (RS-485)
   * USART_CR1_TXEIE USART_ISR_TXE Transmit Data Register Empty
   * USART_CR3_CTSIE USART_ISR_CTS CTS flag                     (not used)
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint16_t ie = priv->ie | USART_CR1_TXEIE;

#  ifdef CONFIG_STM32_SERIALBRK_BSDCOMPAT
      if (priv->ie & USART_CR1_IE_BREAK_INPROGRESS)
        {
          leave_critical_section(flags);
          return;
        }
#  endif

      stm32serial_restoreusartint(priv, ie);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      stm32serial_restoreusartint(priv, priv->ie & ~USART_CR1_TXEIE);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32serial_txready
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/

static bool stm32serial_txready(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;

  return ((stm32serial_getreg(priv, STM32_USART_ISR_OFFSET) &
           USART_ISR_TXE) != 0);
}

/****************************************************************************
 * Name: stm32serial_txempty
 *
 * Description:
 *   Return true only when the final byte has left the USART shift register.
 *
 ****************************************************************************/

static bool stm32serial_txempty(struct uart_dev_s *dev)
{
  struct stm32_serial_s *priv =
    (struct stm32_serial_s *)dev->priv;

#ifdef STM32_USART1_TXDMA
  if (priv->txdma_active)
    {
      return false;
    }
#endif

  return ((stm32serial_getreg(priv, STM32_USART_ISR_OFFSET) &
           USART_ISR_TC) != 0);
}

/****************************************************************************
 * Name: stm32serial_pmprepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int stm32serial_pmprepare(struct pm_callback_s *cb, int domain,
                                   enum pm_state_e pmstate)
{
  int n;
  int ret = OK;
  irqstate_t flags;

  UNUSED(cb);

  if (domain != PM_IDLE_DOMAIN ||
      pmstate == PM_NORMAL || pmstate == PM_IDLE)
    {
      return OK;
    }

  if (pmstate != PM_STANDBY && pmstate != PM_SLEEP)
    {
      return -EINVAL;
    }

  for (n = 0; n < STM32_NUSART; n++)
    {
      struct stm32_serial_s *priv = g_uart_devs[n];

      if (priv == NULL)
        {
          continue;
        }

      flags = spin_lock_irqsave(&priv->lock);
      if (priv->initialized)
        {
          if (stm32serial_busy(priv) ||
              priv->dev.recv.head != priv->dev.recv.tail)
            {
              ret = -EBUSY;
            }
          else
            {
              /* Stop clocks, DMA retention and RX wakeup are not supported.
               * Do not stop service in a void notify callback.
               */

              ret = -ENOTSUP;
            }
        }

      spin_unlock_irqrestore(&priv->lock, flags);
      if (ret != OK)
        {
          return ret;
        }
    }

  return ret;
}
#endif

#endif /* HAVE_UART */
#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void arm_earlyserialinit(void)
{
#ifdef HAVE_UART
  unsigned i;

  /* Disable all USART interrupts */

  for (i = 0; i < STM32_NUSART; i++)
    {
      if (g_uart_devs[i])
        {
          stm32serial_disableusartint(g_uart_devs[i], NULL);
        }
    }

  /* Configure whichever one is the console */

#if CONSOLE_UART > 0
  {
    int ret = stm32serial_setup(&g_uart_devs[CONSOLE_UART - 1]->dev);

    if (ret < 0)
      {
        _err("ERROR: Early serial setup failed: %d\n", ret);
        PANIC();
      }
  }
#endif

#endif /* HAVE UART */
}
#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef HAVE_UART
  char devname[16];
  unsigned i;
  unsigned minor = 0;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Register to receive power management callbacks */

#ifdef CONFIG_PM
  ret = pm_register(&g_serialpm);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

  /* Register the console */

#if CONSOLE_UART > 0
  uart_register("/dev/console", &g_uart_devs[CONSOLE_UART - 1]->dev);

#ifndef CONFIG_STM32_SERIAL_DISABLE_REORDERING
  /* If not disabled, register the console UART to ttyS0 and exclude
   * it from initializing it further down
   */

  uart_register("/dev/ttyS0", &g_uart_devs[CONSOLE_UART - 1]->dev);
  minor = 1;
#endif

#endif /* CONSOLE_UART > 0 */

  /* Register all remaining USARTs */

  strlcpy(devname, "/dev/ttySx", sizeof(devname));

  for (i = 0; i < STM32_NUSART; i++)
    {
      /* Don't create a device for non-configured ports. */

      if (g_uart_devs[i] == 0)
        {
          continue;
        }

#ifndef CONFIG_STM32_SERIAL_DISABLE_REORDERING
      /* Don't create a device for the console - we did that above */

      if (g_uart_devs[i]->dev.isconsole)
        {
          continue;
        }
#endif

      /* Register USARTs as devices in increasing order */

      devname[9] = '0' + minor++;
      uart_register(devname, &g_uart_devs[i]->dev);
    }
#endif /* HAVE UART */
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#if CONSOLE_UART > 0
  struct stm32_serial_s *priv = g_uart_devs[CONSOLE_UART - 1];
  irqstate_t flags;
  uint16_t ie;

  flags = spin_lock_irqsave(&priv->lock);
  ie = priv->ie;
  stm32serial_setusartint(priv, 0);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  stm32serial_setusartint(priv, ie);
  spin_unlock_irqrestore(&priv->lock, flags);
#endif
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#if CONSOLE_UART > 0
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
#endif
}

#endif /* USE_SERIALDRIVER */
