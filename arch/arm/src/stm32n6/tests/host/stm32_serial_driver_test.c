/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_serial_driver_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FAR
#define CONFIG_STM32_STM32N6XXXX
#define CONFIG_STM32_USART1_SERIALDRIVER
#define CONFIG_SERIAL_TERMIOS
#define CONFIG_USART_ERRINTS
#define CONFIG_PM
#define CONFIG_STM32_PM_SERIAL_ACTIVITY 0
#define CONSOLE_UART 1
#define STM32_NUSART 1
#define STM32_HSI_FREQUENCY 64000000
#define USART_CR1_USED_INTS \
  (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)
#define USART_UNCONFIGURE_RX 1
#define USART_UNCONFIGURE_TX 2
#define OK 0
#define UNUSED(value) (void)(value)
#define DEBUGASSERT assert
#define GPIO_MODE_MASK 3
#define GPIO_OUTPUT 1
#define PM_IDLE_DOMAIN 0
#define TCGETS 1
#define TCSETS 2
#define _err test_log
#define _warn test_log

/* ACK_TIMEOUT */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include NUTTX_TERMIOS_HEADER
#include "hardware/stm32n6xxx_rcc.h"
#include "stm32_serial_format.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef unsigned int irqstate_t;
typedef unsigned int spinlock_t;
typedef void *DMA_HANDLE;

struct uart_buffer_s
{
  unsigned int head;
  unsigned int tail;
  unsigned int size;
  char *buffer;
};

struct uart_dev_s
{
  struct uart_buffer_s recv;
  struct uart_buffer_s xmit;
  void *priv;
};

struct inode
{
  void *i_private;
};

struct file
{
  struct inode *f_inode;
};

struct pm_callback_s
{
  int unused;
};

enum pm_state_e
{
  PM_NORMAL, PM_IDLE, PM_STANDBY, PM_SLEEP
};

/* DRIVER_TYPES */;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32serial_receive(struct uart_dev_s *dev, unsigned int *status);
static bool stm32serial_rxavailable(struct uart_dev_s *dev);
static void stm32serial_rxint(struct uart_dev_s *dev, bool enable);
static void stm32serial_send(struct uart_dev_s *dev, int ch);
static bool stm32serial_txready(struct uart_dev_s *dev);
static void uart_recvchars(struct uart_dev_s *dev);
static void uart_xmitchars(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[512];
static char g_txbuffer[512];
static struct stm32_serial_s g_priv =
{
  .dev =
    {
      .recv =
        {
          .size = sizeof(g_rxbuffer),
          .buffer = g_rxbuffer
        },
      .xmit =
        {
          .size = sizeof(g_txbuffer),
          .buffer = g_txbuffer
        },
      .priv = &g_priv
    },
  .bits = 8,
  .baud = 115200,
  .usartbase = STM32_USART1_BASE,
  .tx_gpio = 10,
  .rx_gpio = 11,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rts_gpio = 12,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .cts_gpio = 13,
#endif
  .unconfigure = USART_UNCONFIGURE_RX | USART_UNCONFIGURE_TX
};

static struct stm32_serial_s *g_uart_devs[] =
{
  &g_priv
};

static uint32_t g_regs[12];
static uint32_t g_hsi;
static unsigned int g_delays;
static unsigned int g_errors;
static unsigned int g_writes;
static unsigned int g_releases;
static unsigned int g_disable_count;
static unsigned int g_enable_failure;
static unsigned int g_lock_depth;
static bool g_disable_failure;
static bool g_clocked;
static bool g_pause_rx;
static unsigned int g_txcount;
static uint8_t g_txbytes[512];
static unsigned int g_received;
static unsigned int g_status[512];
static unsigned int g_fifohead;
static unsigned int g_fifolen;
static uint32_t g_fifoerrors[512];
#ifdef STM32_USART1_TXDMA
static int g_dma_stop_result;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static irqstate_t spin_lock_irqsave(spinlock_t *lock)
{
  irqstate_t previous = g_lock_depth;

  assert(*lock == 0);
  *lock = 1;
  g_lock_depth++;
  return previous;
}

static void spin_unlock_irqrestore(spinlock_t *lock, irqstate_t flags)
{
  assert(*lock == 1);
  *lock = 0;
  g_lock_depth = flags;
}

static irqstate_t enter_critical_section(void)
{
  return g_lock_depth++;
}

static void leave_critical_section(irqstate_t flags)
{
  g_lock_depth = flags;
}

static void test_log(const char *format, ...)
{
  UNUSED(format);
  g_errors++;
}

static void up_udelay(unsigned int usecs)
{
  g_delays += usecs;
}

static uint32_t getreg32(uint32_t address)
{
  unsigned int offset;

  if (address == STM32_RCC_HSICFGR)
    {
      return g_hsi;
    }

  assert(g_clocked);
  assert(address >= STM32_USART1_BASE);
  offset = address - STM32_USART1_BASE;
  assert(offset < sizeof(g_regs));
  if (offset == STM32_USART_ISR_OFFSET && g_fifohead < g_fifolen)
    {
      return g_regs[offset / 4] | USART_ISR_RXNE | g_fifoerrors[g_fifohead];
    }

  if (offset == STM32_USART_RDR_OFFSET)
    {
      assert(g_fifohead < g_fifolen);
      return 0xff - g_fifohead++;
    }

  return g_regs[offset / 4];
}

static void putreg32(uint32_t value, uint32_t address)
{
  uint32_t oldcr1 = g_regs[STM32_USART_CR1_OFFSET / 4];
  unsigned int offset;

  g_writes++;
  if (address == STM32_RCC_APB2ENSR)
    {
      assert(value == RCC_APB2ENR_USART1EN);
      g_clocked = true;
      return;
    }

  if (address == STM32_RCC_APB2ENCR)
    {
      assert(value == RCC_APB2ENR_USART1EN);
      assert((oldcr1 & (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE)) == 0);
      assert((g_regs[STM32_USART_ISR_OFFSET / 4] &
              (USART_ISR_TEACK | USART_ISR_REACK)) == 0);
      g_clocked = false;
      return;
    }

  assert(g_clocked);
  offset = address - STM32_USART1_BASE;
  assert(offset < sizeof(g_regs));
  if (offset == STM32_USART_CR1_OFFSET)
    {
      if (((oldcr1 ^ value) &
           (USART_CR1_FIFOEN | USART_CR1_FORMAT_MASK)) != 0)
        {
          assert((oldcr1 & USART_CR1_UE) == 0);
          assert((value & USART_CR1_UE) == 0);
        }

      if ((value & USART_CR1_UE) == 0)
        {
          g_disable_count++;
          if (!g_disable_failure)
            {
              g_regs[STM32_USART_ISR_OFFSET / 4] =
                USART_ISR_TC | USART_ISR_TXE;
            }
        }
      else if ((oldcr1 & USART_CR1_UE) == 0)
        {
          g_regs[STM32_USART_ISR_OFFSET / 4] = USART_ISR_TXE;
          if (g_enable_failure != 0)
            {
              g_enable_failure--;
            }
          else
            {
              g_regs[STM32_USART_ISR_OFFSET / 4] |=
                ((value & USART_CR1_TE) != 0 ? USART_ISR_TEACK : 0) |
                ((value & USART_CR1_RE) != 0 ? USART_ISR_REACK : 0);
            }
        }
    }

  if (offset == STM32_USART_BRR_OFFSET ||
      offset == STM32_USART_PRESC_OFFSET ||
      offset == STM32_USART_CR2_OFFSET)
    {
      assert((oldcr1 & USART_CR1_UE) == 0);
    }

  if (offset == STM32_USART_ICR_OFFSET)
    {
      g_regs[STM32_USART_ISR_OFFSET / 4] &= ~value;
      if (g_fifohead < g_fifolen)
        {
          g_fifoerrors[g_fifohead] &= ~value;
        }
    }

  if (offset == STM32_USART_TDR_OFFSET)
    {
      assert((oldcr1 & (USART_CR1_UE | USART_CR1_TE)) ==
             (USART_CR1_UE | USART_CR1_TE));
      assert(g_txcount < sizeof(g_txbytes));
      g_txbytes[g_txcount++] = value;
      g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_TC;
    }

  g_regs[offset / 4] = value;
}

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static int stm32_configgpio(uint32_t pin)
{
  assert(pin != 0);
  return 0;
}
#endif

static void stm32_unconfiggpio(uint32_t pin)
{
  assert(pin != 0);
  g_releases++;
}

#ifdef STM32_USART1_TXDMA
static int stm32_dmastop(DMA_HANDLE handle)
{
  assert(handle != NULL);
  return g_dma_stop_result;
}

static int stm32_dmafree(DMA_HANDLE handle)
{
  assert(handle != NULL);
  return 0;
}
#endif

speed_t cfgetspeed(const struct termios *termiosp)
{
  return termiosp->c_speed;
}

int cfsetspeed(struct termios *termiosp, speed_t speed)
{
  termiosp->c_speed = speed;
  return 0;
}

static void arm_lowputc(char ch)
{
  assert(g_lock_depth != 0);
  putreg32((uint8_t)ch, STM32_USART1_TDR);
}

/* DRIVER_ROUTINES */

static void uart_recvchars(struct uart_dev_s *dev)
{
  if (g_pause_rx)
    {
      stm32serial_rxint(dev, false);
      return;
    }

  while (stm32serial_rxavailable(dev))
    {
      unsigned int status;
      int ch = stm32serial_receive(dev, &status);

      assert(ch == (int)((0xff - g_received) &
                         (g_priv.bits == 7 ? 0x7f : 0xff)));
      g_status[g_received++] = status;
    }
}

static void uart_xmitchars(struct uart_dev_s *dev)
{
  while (dev->xmit.head != dev->xmit.tail && stm32serial_txready(dev))
    {
      stm32serial_send(dev, dev->xmit.buffer[dev->xmit.tail++]);
    }

  stm32serial_setusartint(&g_priv, g_priv.ie & ~USART_CR1_TXEIE);
}

static void reset(void)
{
  memset(g_regs, 0, sizeof(g_regs));
  memset(g_fifoerrors, 0, sizeof(g_fifoerrors));
  g_regs[STM32_USART_ISR_OFFSET / 4] = USART_ISR_TC | USART_ISR_TXE;
  g_hsi = 0;
  g_clocked = true;
  g_disable_failure = false;
  g_enable_failure = 0;
  g_disable_count = g_writes = g_delays = g_errors = g_releases = 0;
  g_txcount = g_fifohead = g_fifolen = g_received = 0;
  g_pause_rx = false;
  g_priv.initialized = false;
  g_priv.shutdown_error = 0;
  g_priv.baud = 115200;
  g_priv.bits = 8;
  g_priv.parity = 0;
  g_priv.stopbits2 = false;
  g_priv.ie = 0;
  g_priv.dev.xmit.head = g_priv.dev.xmit.tail = 0;
  g_priv.dev.recv.head = g_priv.dev.recv.tail = 0;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  g_priv.iflow = false;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  g_priv.oflow = false;
#endif
#ifdef STM32_USART1_TXDMA
  g_priv.txdma = NULL;
  g_priv.txdma_active = false;
#endif
  assert(g_priv.lock == 0 && g_lock_depth == 0);
}

static int ioctl_termios(int command, struct termios *termiosp)
{
  struct inode inode =
  {
    .i_private = &g_priv.dev
  };

  struct file file =
  {
    .f_inode = &inode
  };

  return stm32serial_ioctl(&file, command, (unsigned long)termiosp);
}

static void configure(void)
{
  struct stm32_usart_format_s format;

  assert(stm32_usart_format(stm32_usart_clock(), g_priv.baud, g_priv.bits,
                            g_priv.parity, g_priv.stopbits2, &format) == 0);
  assert(stm32_usart_configure(g_priv.usartbase, &format, 0) == 0);
  assert(stm32serial_setup(&g_priv.dev) == 0);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
}

static void test_handoff(void)
{
  struct stm32_usart_format_s format;
  unsigned int disables;

  reset();
  for (unsigned int divider = 0; divider < 4; divider++)
    {
      g_hsi = divider << RCC_HSICFGR_HSIDIV_SHIFT;
      assert(stm32_usart_clock() == (64000000u >> divider));
    }

  g_regs[STM32_USART_CR1_OFFSET / 4] =
    USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_PCE;
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TEACK | USART_ISR_REACK;
  g_regs[STM32_USART_PRESC_OFFSET / 4] = 7;
  g_regs[STM32_USART_BRR_OFFSET / 4] = 99;
  configure();
  assert(g_disable_count != 0);
  assert(g_regs[STM32_USART_CR1_OFFSET / 4] & USART_CR1_FIFOEN);
  assert(g_regs[STM32_USART_PRESC_OFFSET / 4] == 0);
  assert(g_regs[STM32_USART_BRR_OFFSET / 4] == 69);
  assert(stm32_usart_format(stm32_usart_clock(), 115200, 8, 0, false,
                            &format) == 0);
  g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_TC;
  disables = g_disable_count;
  assert(stm32_usart_configure(g_priv.usartbase, &format, 0) == 0);
  assert(stm32serial_setup(&g_priv.dev) == 0);
  assert(g_disable_count == disables);
  g_regs[STM32_USART_ISR_OFFSET / 4] &=
    ~(USART_ISR_TEACK | USART_ISR_REACK);
  g_delays = 0;
  assert(stm32_usart_configure(g_priv.usartbase, &format, 0) == -ETIMEDOUT);
  assert(g_delays == USART_ACK_TIMEOUT_US);
  assert(g_disable_count == disables);
}

static void test_formats(void)
{
  struct termios requested =
  {
    0
  };

  struct termios reported =
  {
    0
  };

  unsigned int bits;
  unsigned int parity;
  unsigned int stop;
  uint32_t before[12];

  reset();
  configure();
  requested.c_cflag = CS7;
  cfsetispeed(&requested, 100000);
#ifdef CONFIG_SUPPRESS_UART_CONFIG
  memcpy(before, g_regs, sizeof(before));
  assert(ioctl_termios(TCSETS, &requested) == -ENOTSUP);
  assert(memcmp(before, g_regs, sizeof(before)) == 0);
  UNUSED(reported);
  UNUSED(bits);
  UNUSED(parity);
  UNUSED(stop);
#else
  for (bits = 7; bits <= 8; bits++)
    {
      for (parity = 0; parity < 3; parity++)
        {
          for (stop = 0; stop < 2; stop++)
            {
              g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
              requested.c_cflag = (bits == 7 ? CS7 : CS8) |
                (parity != 0 ? PARENB : 0) | (parity == 1 ? PARODD : 0) |
                (stop != 0 ? CSTOPB : 0);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
              requested.c_cflag |= CRTS_IFLOW;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
              requested.c_cflag |= CCTS_OFLOW;
#endif
              stm32serial_rxint(&g_priv.dev, true);
              uint16_t ie = g_priv.ie;

              assert(ioctl_termios(TCSETS, &requested) == 0);
              assert(g_priv.ie == ie);
              assert(ioctl_termios(TCGETS, &reported) == 0);
              assert(reported.c_cflag == requested.c_cflag);
              assert(cfgetispeed(&reported) == 100000);
              assert(cfgetospeed(&reported) == 100000);
              assert(g_priv.bits == bits && g_priv.parity == parity);
              stm32serial_send(&g_priv.dev, 0xff);
              assert(g_txbytes[g_txcount - 1] == (bits == 7 ? 0x7f : 0xff));
            }
        }
    }

  memcpy(before, g_regs, sizeof(before));
  uint32_t oldbaud = g_priv.baud;
  uint8_t oldbits = g_priv.bits;

  cfsetispeed(&requested, B0);
  assert(ioctl_termios(TCSETS, &requested) == -EINVAL);
  cfsetispeed(&requested, UINT32_MAX);
  assert(ioctl_termios(TCSETS, &requested) == -ERANGE);
  cfsetispeed(&requested, 115200);
  requested.c_cflag = CS6;
  assert(ioctl_termios(TCSETS, &requested) == -EINVAL);
  assert(ioctl_termios(TCSETS, NULL) == -EINVAL);
  assert(memcmp(before, g_regs, sizeof(before)) == 0);
  assert(g_priv.baud == oldbaud && g_priv.bits == oldbits);

#if !defined(CONFIG_SERIAL_IFLOWCONTROL) || !defined(CONFIG_SERIAL_OFLOWCONTROL)
  requested.c_cflag = CS8 | CRTSCTS;
  assert(ioctl_termios(TCSETS, &requested) == -EINVAL);
#endif
#endif
}

static void test_busy_and_rollback(void)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct termios requested =
  {
    .c_cflag = CS7,
    .c_speed = 57600
  };

  uint32_t before[12];
  uint32_t oldbaud;

  reset();
  configure();
  stm32serial_rxint(&g_priv.dev, true);
  memcpy(before, g_regs, sizeof(before));
  oldbaud = g_priv.baud;
  g_priv.dev.xmit.head = 1;
  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  g_priv.dev.xmit.head = 0;
#ifdef STM32_USART1_TXDMA
  g_priv.txdma_active = true;
  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  g_priv.txdma_active = false;
#endif
  g_regs[STM32_USART_CR3_OFFSET / 4] |= USART_CR3_DMAR;
  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  g_regs[STM32_USART_CR3_OFFSET / 4] &= ~USART_CR3_DMAR;
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_BUSY;
  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_BUSY;
  g_fifolen = 1;
  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  g_fifolen = 0;
  g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_TC;
  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  assert(memcmp(before, g_regs, sizeof(before)) == 0);
  assert(g_priv.baud == oldbaud && g_priv.bits == 8);

  g_disable_failure = true;
  assert(ioctl_termios(TCSETS, &requested) == -ETIMEDOUT);
  assert(g_delays == USART_ACK_TIMEOUT_US);
  assert(g_regs[STM32_USART_CR1_OFFSET / 4] == before[0]);
  g_disable_failure = false;
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  g_delays = 0;
  g_enable_failure = 1;
  assert(ioctl_termios(TCSETS, &requested) == -ETIMEDOUT);
  assert(g_delays == USART_ACK_TIMEOUT_US);
  assert(g_priv.baud == oldbaud && g_priv.bits == 8);
  for (unsigned int i = 0; i < 4; i++)
    {
      assert(g_regs[i] == before[i]);
    }

  assert(g_priv.ie == (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR3_EIE));
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  g_priv.dev.recv.head = 7;
  g_priv.dev.recv.tail = 2;
  assert(ioctl_termios(TCSETS, &requested) == 0);
  assert(g_priv.dev.recv.head == 7 && g_priv.dev.recv.tail == 2);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  requested.c_speed = 38400;
  g_enable_failure = 2;
  g_delays = 0;
  assert(ioctl_termios(TCSETS, &requested) == -EIO);
  assert(g_delays == 2 * USART_ACK_TIMEOUT_US);
  assert(g_priv.baud == 57600 && g_priv.bits == 7);
#endif
}

static void test_fifo_and_debug(void)
{
  static const uint32_t errors[] =
  {
    USART_ISR_PE, USART_ISR_FE, USART_ISR_NF, USART_ISR_ORE, 0
  };

  reset();
  configure();
  g_priv.bits = 7;
  stm32serial_rxint(&g_priv.dev, true);
  for (unsigned int length = 1; length <= 17; length++)
    {
      g_fifohead = g_received = 0;
      g_fifolen = length;
      for (unsigned int i = 0; i < length; i++)
        {
          g_fifoerrors[i] = errors[i % 5];
        }

      assert(stm32serial_interrupt(0, NULL, &g_priv) == 0);
      assert(g_received == length);
      for (unsigned int i = 0; i < length; i++)
        {
          assert((g_status[i] >> 16 & USART_ISR_ERRORS) == errors[i % 5]);
        }
    }

  g_fifolen = g_fifohead = 0;
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_ERRORS;
  assert(stm32serial_interrupt(0, NULL, &g_priv) == 0);
  assert((g_regs[STM32_USART_ISR_OFFSET / 4] & USART_ISR_ERRORS) == 0);

  g_fifohead = g_received = 0;
  g_fifolen = 1;
  g_fifoerrors[0] = USART_ISR_PE;
  g_pause_rx = true;
  assert(stm32serial_interrupt(0, NULL, &g_priv) == 0);
  assert(g_fifoerrors[0] == USART_ISR_PE && g_received == 0);
  g_pause_rx = false;
  stm32serial_rxint(&g_priv.dev, true);
  assert(stm32serial_interrupt(0, NULL, &g_priv) == 0);
  assert((g_status[0] >> 16 & USART_ISR_PE) != 0);

  g_txcount = 0;
  uint16_t ie = g_priv.ie;

  up_putc('\n');
  assert(g_txcount == 2 && g_txbytes[0] == '\r' && g_txbytes[1] == '\n');
  assert(g_priv.ie == ie && g_priv.lock == 0);
  assert(!stm32serial_txempty(&g_priv.dev));
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  assert(stm32serial_txempty(&g_priv.dev));
#ifdef STM32_USART1_TXDMA
  g_priv.txdma_active = true;
  assert(!stm32serial_txempty(&g_priv.dev));
  g_priv.txdma_active = false;
#endif
}

static void test_pm_and_close(void)
{
  uint32_t before[12];

  reset();
  configure();
  stm32serial_rxint(&g_priv.dev, true);
  memcpy(before, g_regs, sizeof(before));
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_IDLE) == 0);
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_SLEEP) == -ENOTSUP);
  g_priv.dev.recv.head = 1;
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_SLEEP) == -EBUSY);
  g_priv.dev.recv.head = 0;
  g_priv.dev.xmit.head = 1;
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_SLEEP) == -EBUSY);
  g_priv.dev.xmit.head = 0;
  g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_TC;
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_STANDBY) == -EBUSY);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
#ifdef STM32_USART1_TXDMA
  g_priv.txdma_active = true;
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_SLEEP) == -EBUSY);
  g_priv.txdma_active = false;
#endif
  assert(memcmp(before, g_regs, sizeof(before)) == 0);
  assert(g_delays == 0);

  g_disable_failure = true;
  stm32serial_shutdown(&g_priv.dev);
  assert(g_clocked && g_priv.initialized && g_errors != 0);
  assert(stm32serial_setup(&g_priv.dev) == -ETIMEDOUT);
  g_disable_failure = false;
  stm32serial_shutdown(&g_priv.dev);
  assert(!g_clocked && !g_priv.initialized && g_releases >= 2);
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_SLEEP) == 0);
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  assert(stm32serial_setup(&g_priv.dev) == 0);
  assert(g_clocked && g_priv.initialized);
#endif

#ifdef STM32_USART1_TXDMA
  reset();
  configure();
  g_priv.txdma = &g_priv;
  g_priv.txdma_active = true;
  g_dma_stop_result = -ETIMEDOUT;
  stm32serial_shutdown(&g_priv.dev);
  assert(g_clocked && g_priv.initialized && g_priv.txdma_active);
  assert(stm32serial_setup(&g_priv.dev) == -ETIMEDOUT);
  assert(g_errors != 0);
  g_dma_stop_result = 0;
  stm32serial_shutdown(&g_priv.dev);
  assert(!g_clocked && !g_priv.initialized && g_priv.txdma == NULL);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  test_handoff();
  test_formats();
  test_busy_and_rollback();
  test_fifo_and_debug();
  test_pm_and_close();
  assert(g_priv.lock == 0 && g_lock_depth == 0);
  return 0;
}
