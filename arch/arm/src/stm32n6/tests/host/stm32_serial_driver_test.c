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
#define STM32_NUART 0
#define STM32_HSI_FREQUENCY 64000000
#define USART_CR1_USED_INTS \
  (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)
#define USART_UNCONFIGURE_RX 1
#define USART_UNCONFIGURE_TX 2
#define OK 0
#define UNUSED(value) (void)(value)
#define DEBUGASSERT assert
/* GPIO_DEFINITIONS */
#define PM_IDLE_DOMAIN 0
#define TCGETS 1
#define TCSETS 2
#define TIOCSINVERT 3
#define TIOCSSINGLEWIRE 4
#define TCFLSH 5
#define TCIFLUSH 0
#define TCOFLUSH 1
#define TCIOFLUSH 2
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define USART_TXDMA_BLOCK_MAX 65535
#define USART_DEBUG_BUFSIZE 128
#define uart_dmatxavail(dev) stm32serial_dmatxavail(dev)
#define uart_dmasend(dev) stm32serial_dmasend(dev)
/* IOCTL_FLAGS */
#define uart_enablerxint(dev) stm32serial_rxint(dev, true)
#define uart_disablerxint(dev) stm32serial_rxint(dev, false)
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
#include "stm32_dma.h"

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

struct uart_dmaxfer_s
{
  char *buffer;
  char *nbuffer;
  size_t length;
  size_t nlength;
  size_t nbytes;
};

struct uart_dev_s
{
  struct uart_buffer_s recv;
  struct uart_buffer_s xmit;
  void *priv;
  bool isconsole;
  struct uart_dmaxfer_s dmatx;
};

typedef struct uart_dev_s uart_dev_t;

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
static void stm32serial_txint(struct uart_dev_s *dev, bool enable);
static void stm32serial_detach(struct uart_dev_s *dev);
#ifdef STM32_SERIAL_TXDMA
static void stm32serial_dmasend(struct uart_dev_s *dev);
static void stm32serial_dmatxavail(struct uart_dev_s *dev);
static void stm32serial_dmatxcallback(DMA_HANDLE handle, uint8_t status,
                                     void *arg);
static int stm32serial_dmaabort(struct stm32_serial_s *priv, bool discard);
static void stm32serial_dmafallback(struct stm32_serial_s *priv, int error);
static bool stm32serial_debugsend(struct stm32_serial_s *priv);
static void uart_xmitchars_dma(struct uart_dev_s *dev);
static void uart_xmitchars_done(struct uart_dev_s *dev);
#endif
static void uart_recvchars(struct uart_dev_s *dev);
static void uart_xmitchars(struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool stm32serial_rxflowcontrol(struct uart_dev_s *dev,
                                      unsigned int buffered, bool upper);
#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void stm32serial_setflow(struct stm32_serial_s *priv);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[512];
static char g_txbuffer[512];
static const struct stm32_usart_s g_config =
{
  .base = STM32_USART1_BASE,
  .clock = STM32_HSI_FREQUENCY,
  .tx_gpio = GPIO_ALT | GPIO_AF7 | 10,
  .rx_gpio = GPIO_ALT | GPIO_AF7 | 11,
  .rts_gpio = GPIO_ALT | GPIO_AF7 | 12,
  .cts_gpio = GPIO_ALT | GPIO_AF7 | 13,
  .enable = STM32_RCC_APB2ENSR,
  .disable = STM32_RCC_APB2ENCR,
  .resetset = STM32_RCC_APB2RSTSR,
  .resetclear = STM32_RCC_APB2RSTCR,
  .lpen = STM32_RCC_APB2LPENSR,
  .lpdisable = STM32_RCC_APB2LPENCR,
  .rcc_bit = RCC_APB2ENR_USART1EN,
  .selector = STM32_RCC_CCIPR13,
  .selmask = RCC_CCIPR13_USART1SEL_MASK,
  .selsource = RCC_CCIPR13_USART1SEL_HSI
};

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
      .priv = &g_priv,
      .isconsole = true
    },
  .bits = 8,
  .baud = 115200,
  .config = &g_config,
  .unconfigure = USART_UNCONFIGURE_RX | USART_UNCONFIGURE_TX
};

static struct stm32_serial_s *g_uart_devs[] =
{
  &g_priv
};

static uint32_t g_regs[12];
static uint32_t g_hsi;
static uint32_t g_selector;
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
static uint8_t g_txbytes[140000];
static unsigned int g_received;
static uint32_t g_gpio[16];
static unsigned int g_gpio_failure;
static bool g_rts_level;
static unsigned int g_status[512];
static unsigned int g_fifohead;
static unsigned int g_fifolen;
static uint32_t g_fifoerrors[512];
#ifdef STM32_SERIAL_TXDMA
static int g_dma_stop_result;
static int g_dma_setup_result;
static int g_dma_start_result;
static int g_dma_progress_result;
static int g_dma_callback_result;
static int g_dma_free_result;
static unsigned int g_irq_detaches;
static bool g_dma_available;
static size_t g_dma_transferred;
static struct stm32_dma_config_s g_dma_config;
static char g_debugbuffer[USART_DEBUG_BUFSIZE];
static unsigned int g_debughead;
static unsigned int g_debugtail;
static bool g_debugoverflow;
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

  if (address == STM32_RCC_CCIPR13)
    {
      return g_selector;
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
  if (address == STM32_RCC_CCIPR13)
    {
      assert((oldcr1 & USART_CR1_UE) == 0 ||
             (value & g_config.selmask) == (g_selector & g_config.selmask));
      g_selector = value;
      return;
    }

  if (address == STM32_RCC_APB2LPENSR ||
      address == STM32_RCC_APB2LPENCR)
    {
      assert(value == RCC_APB2ENR_USART1EN);
      return;
    }

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

      if ((value & USART_CR1_TE) == 0)
        {
            g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
        }
    }

  if (offset == STM32_USART_BRR_OFFSET ||
      offset == STM32_USART_PRESC_OFFSET ||
      offset == STM32_USART_CR2_OFFSET)
    {
      assert((oldcr1 & USART_CR1_UE) == 0);
    }

  if (offset == STM32_USART_CR3_OFFSET &&
      ((g_regs[offset / 4] ^ value) &
       (USART_CR3_HDSEL | USART_CR3_RTSE | USART_CR3_CTSE)) != 0)
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
static void modifyreg32(uint32_t address, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(address) & ~clear) | set, address);
}
#endif

int stm32_usart_disable(uint32_t base);

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static int stm32_configgpio(uint32_t pin)
{
  assert(pin != 0);
  if (g_gpio_failure != 0)
    {
      g_gpio_failure--;
      return -EIO;
    }

  g_gpio[pin & 15] = pin;
  return 0;
}
#endif

#if defined(CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS) && \
    defined(CONFIG_STM32_FLOWCONTROL_BROKEN)
static void stm32_gpiowrite(uint32_t pin, bool value)
{
  assert(pin == g_config.rts_gpio);
  g_rts_level = value;
}
#endif

static void stm32_unconfiggpio(uint32_t pin)
{
  assert(pin != 0);
  g_releases++;
}

#ifdef STM32_SERIAL_TXDMA
DMA_HANDLE stm32_dmachannel(const struct stm32_dma_request_s *request)
{
  assert(request->controller == STM32_DMA_CONTROLLER_GPDMA1);
  assert(request->peripheral_address ==
         g_config.base + STM32_USART_TDR_OFFSET);
  return g_dma_available ? &g_priv : NULL;
}

int stm32_dmacallback(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  assert(handle != NULL && callback == stm32serial_dmatxcallback);
  assert(arg == &g_priv);
  return g_dma_callback_result;
}

int stm32_dmasetup(DMA_HANDLE handle,
                   const struct stm32_dma_config_s *config)
{
  assert(handle != NULL && config->width == 1);
  assert(config->nbytes <= USART_TXDMA_BLOCK_MAX);
  assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BATCH);
  g_dma_config = *config;
  return g_dma_setup_result;
}

int stm32_dmastart(DMA_HANDLE handle)
{
  assert(handle != NULL);
  assert((g_regs[STM32_USART_CR3_OFFSET / 4] & USART_CR3_DMAT) == 0);
  assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BATCH);
  g_dma_transferred = 0;
  return g_dma_start_result;
}

int stm32_dmastop(DMA_HANDLE handle)
{
  assert(handle != NULL);
  return g_dma_stop_result;
}

int stm32_dmaabort(DMA_HANDLE handle, size_t *transferred)
{
  assert(handle != NULL);
  assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BLOCK);
  if (g_dma_progress_result != 0)
    {
      return g_dma_progress_result;
    }

  if (g_dma_stop_result == 0)
    {
      *transferred = g_dma_transferred;
    }

  return g_dma_stop_result;
}

int stm32_dmafree(DMA_HANDLE handle)
{
  assert(handle != NULL);
  return g_dma_free_result;
}

static void uart_datasent(struct uart_dev_s *dev)
{
  assert(dev == &g_priv.dev);
}
#endif

static void up_disable_irq(int irq)
{
  UNUSED(irq);
}

static void irq_detach(int irq)
{
  UNUSED(irq);
#ifdef STM32_SERIAL_TXDMA
  g_irq_detaches++;
#endif
}

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
      stm32serial_send(dev, dev->xmit.buffer[dev->xmit.tail]);
      dev->xmit.tail = (dev->xmit.tail + 1) % dev->xmit.size;
    }

  stm32serial_setusartint(&g_priv, g_priv.ie & ~USART_CR1_TXEIE);
}

static void reset(void)
{
  memset(g_regs, 0, sizeof(g_regs));
  memset(g_fifoerrors, 0, sizeof(g_fifoerrors));
  g_regs[STM32_USART_ISR_OFFSET / 4] = USART_ISR_TC | USART_ISR_TXE;
  g_hsi = 0;
  g_selector = g_config.selsource;
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
  g_priv.config = &g_config;
  g_priv.ie = 0;
  g_priv.dev.xmit.head = g_priv.dev.xmit.tail = 0;
  g_priv.dev.recv.head = g_priv.dev.recv.tail = 0;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  g_priv.iflow = false;
  g_priv.rxthrottled = false;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  g_priv.oflow = false;
#endif
  memset(g_gpio, 0, sizeof(g_gpio));
  g_gpio_failure = 0;
  g_rts_level = false;
#ifdef STM32_SERIAL_TXDMA
  g_priv.txdma = NULL;
  g_priv.txdma_state = STM32_SERIAL_TXDMA_IDLE;
  g_priv.txdma_fallback = false;
  g_priv.txdma_length = 0;
  g_priv.txdma_was_idle = false;
  g_priv.txdma_error = 0;
  memset(&g_priv.dev.dmatx, 0, sizeof(g_priv.dev.dmatx));
  g_dma_stop_result = g_dma_setup_result = g_dma_start_result = 0;
  g_dma_progress_result = 0;
  g_dma_callback_result = g_dma_free_result = 0;
  g_irq_detaches = 0;
  g_dma_available = true;
  g_dma_transferred = 0;
  g_debughead = g_debugtail = 0;
  g_debugoverflow = false;
#endif
  assert(g_priv.lock == 0 && g_lock_depth == 0);
}

static int ioctl_arg(int command, unsigned long arg)
{
  struct inode inode =
  {
    .i_private = &g_priv.dev
  };

  struct file file =
  {
    .f_inode = &inode
  };

  return stm32serial_ioctl(&file, command, arg);
}

static int ioctl_termios(int command, struct termios *termiosp)
{
  return ioctl_arg(command, (unsigned long)termiosp);
}

static void configure(void)
{
  struct stm32_usart_format_s format;

  assert(stm32_usart_initialize(g_priv.config, false) == 0);
  assert(stm32_usart_format(stm32_usart_clock(g_priv.config),
                            g_priv.baud, g_priv.bits,
                            g_priv.parity, g_priv.stopbits2, &format) == 0);
  assert(stm32_usart_configure(g_priv.config->base, &format, 0) == 0);
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
      assert(stm32_usart_clock(g_priv.config) == (64000000u >> divider));
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
  assert(stm32_usart_format(stm32_usart_clock(g_priv.config),
                            115200, 8, 0, false,
                            &format) == 0);
  g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_TC;
  disables = g_disable_count;
  assert(stm32_usart_configure(g_priv.config->base, &format, 0) == 0);
  assert(stm32serial_setup(&g_priv.dev) == 0);
  assert(g_disable_count == disables);
  g_regs[STM32_USART_ISR_OFFSET / 4] &=
    ~(USART_ISR_TEACK | USART_ISR_REACK);
  g_delays = 0;
  assert(stm32_usart_configure(g_priv.config->base, &format, 0) ==
         -ETIMEDOUT);
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
#ifdef STM32_SERIAL_TXDMA
  g_priv.txdma_state = STM32_SERIAL_TXDMA_BATCH;
  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  g_priv.txdma_state = STM32_SERIAL_TXDMA_IDLE;
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
#ifdef STM32_SERIAL_TXDMA
  g_priv.txdma_state = STM32_SERIAL_TXDMA_BATCH;
  assert(!stm32serial_txempty(&g_priv.dev));
  g_priv.txdma_state = STM32_SERIAL_TXDMA_IDLE;
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
#ifdef STM32_SERIAL_TXDMA
  g_priv.txdma_state = STM32_SERIAL_TXDMA_BATCH;
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_SLEEP) == -EBUSY);
  g_priv.txdma_state = STM32_SERIAL_TXDMA_IDLE;
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

#ifdef STM32_SERIAL_TXDMA
  reset();
  configure();
  g_priv.txdma = &g_priv;
  g_priv.txdma_state = STM32_SERIAL_TXDMA_BATCH;
  g_dma_stop_result = -ETIMEDOUT;
  stm32serial_shutdown(&g_priv.dev);
  assert(g_clocked && g_priv.initialized &&
         g_priv.txdma_state == STM32_SERIAL_TXDMA_BATCH);
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

static void test_initial_flow(void)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct stm32_usart_format_s format;
  uint32_t flow;
  bool iflow = false;
  bool oflow = false;

  reset();
  g_regs[STM32_USART_CR3_OFFSET / 4] = USART_CR3_HDSEL;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  iflow = g_priv.iflow = true;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  oflow = g_priv.oflow = true;
#endif
  assert(stm32_usart_flowcontrol(&g_config, iflow, oflow, &flow) == 0);
  assert(stm32_usart_format(stm32_usart_clock(&g_config), 115200, 8, 0,
                            false, &format) == 0);
  assert(stm32_usart_configure(g_config.base, &format, flow) == 0);
  assert((g_regs[STM32_USART_CR3_OFFSET / 4] & USART_CR3_HDSEL) == 0);
  assert((g_regs[STM32_USART_ISR_OFFSET / 4] & USART_ISR_TC) == 0);
  assert(stm32serial_setup(&g_priv.dev) == 0);
#endif
}

static void test_flow(void)
{
#if !defined(CONFIG_SUPPRESS_UART_CONFIG) && \
    (defined(CONFIG_SERIAL_IFLOWCONTROL) || \
     defined(CONFIG_SERIAL_OFLOWCONTROL))
  struct termios requested =
  {
    .c_cflag = CS8,
    .c_speed = 115200
  };

  struct stm32_usart_s missing = g_config;
  uint32_t before[12];

  reset();
  configure();
  for (unsigned int flow = 0; flow < 4; flow++)
    {
      requested.c_cflag = CS8;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
      requested.c_cflag |= (flow & 1) != 0 ? CRTS_IFLOW : 0;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
      requested.c_cflag |= (flow & 2) != 0 ? CCTS_OFLOW : 0;
#endif
      g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
      assert(ioctl_termios(TCSETS, &requested) == 0);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
      assert(g_priv.iflow == ((requested.c_cflag & CRTS_IFLOW) != 0));
#ifndef CONFIG_STM32_FLOWCONTROL_BROKEN
      assert(((g_regs[2] & USART_CR3_RTSE) != 0) == g_priv.iflow);
#endif
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
      assert(g_priv.oflow == ((requested.c_cflag & CCTS_OFLOW) != 0));
      assert(((g_regs[2] & USART_CR3_CTSE) != 0) == g_priv.oflow);
#endif
    }

  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  memcpy(before, g_regs, sizeof(before));
  missing.rts_gpio = missing.cts_gpio = 0;
  g_priv.config = &missing;
  requested.c_cflag = CS8 | CRTSCTS;
  assert(ioctl_termios(TCSETS, &requested) == -EINVAL);
  g_priv.initialized = false;
  assert(stm32serial_setup(&g_priv.dev) == -EINVAL);
  assert(memcmp(before, g_regs, sizeof(before)) == 0);
  g_priv.initialized = true;
  g_priv.config = &g_config;

#ifdef CONFIG_SERIAL_IFLOWCONTROL
  stm32serial_rxint(&g_priv.dev, true);
  bool stopped = stm32serial_rxflowcontrol(&g_priv.dev, 500, true);

#ifdef CONFIG_STM32_FLOWCONTROL_BROKEN
  assert(!stopped && g_rts_level);
  assert(g_priv.ie & USART_CR1_RXNEIE);
#else
  assert(stopped && (g_priv.ie & USART_CR1_RXNEIE) == 0);
#endif
  assert(!stm32serial_rxflowcontrol(&g_priv.dev, 0, false));
  assert(g_priv.ie & USART_CR1_RXNEIE);
  assert(!g_rts_level);

  stm32serial_rxflowcontrol(&g_priv.dev, 500, true);
  requested.c_cflag = CS8;
  assert(ioctl_termios(TCSETS, &requested) == 0);
  assert(!g_priv.rxthrottled && !g_rts_level);
  assert(g_priv.ie & USART_CR1_RXNEIE);

  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  g_priv.dev.recv.head = g_priv.dev.recv.size - 1;
  requested.c_cflag = CS8 | CRTS_IFLOW;
  assert(ioctl_termios(TCSETS, &requested) == 0);
  assert(g_priv.rxthrottled);
#ifdef CONFIG_STM32_FLOWCONTROL_BROKEN
  assert(g_rts_level && (g_priv.ie & USART_CR1_RXNEIE) != 0);
#else
  assert((g_priv.ie & USART_CR1_RXNEIE) == 0);
#endif
  g_priv.dev.recv.tail = g_priv.dev.recv.size / 2;
  stm32serial_setflow(&g_priv);
  assert(g_priv.rxthrottled);
  g_priv.dev.recv.tail = g_priv.dev.recv.head;
  stm32serial_setflow(&g_priv);
  assert(!g_priv.rxthrottled && (g_priv.ie & USART_CR1_RXNEIE) != 0);
  g_priv.dev.recv.head = g_priv.dev.recv.tail = 0;

#endif

  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  requested.c_cflag = CS8;
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  requested.c_cflag |= CCTS_OFLOW;
#endif
  assert(ioctl_termios(TCSETS, &requested) == 0);
  stm32serial_send(&g_priv.dev, 0x55);
  g_priv.dev.xmit.head = g_priv.dev.xmit.tail;
  assert(!stm32serial_txempty(&g_priv.dev));
  assert(stm32serial_pmprepare(NULL, PM_IDLE_DOMAIN, PM_SLEEP) == -EBUSY);
  unsigned int delays = g_delays;

  assert(ioctl_termios(TCSETS, &requested) == -EBUSY);
  assert(g_delays == delays);
#endif
}

static void test_rc_modes(void)
{
#if defined(CONFIG_STM32_USART_INVERT) || \
    defined(CONFIG_STM32_USART_SINGLEWIRE)
  uint32_t before[12];

  reset();
  configure();
  memcpy(before, g_regs, sizeof(before));
#ifdef CONFIG_SUPPRESS_UART_CONFIG
#ifdef CONFIG_STM32_USART_INVERT
  assert(ioctl_arg(TIOCSINVERT, SER_INVERT_ENABLED_RX) == -ENOTSUP);
#endif
#ifdef CONFIG_STM32_USART_SINGLEWIRE
  assert(ioctl_arg(TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED) == -ENOTSUP);
#endif
  assert(memcmp(before, g_regs, sizeof(before)) == 0);
#else
  struct termios requested =
  {
    .c_cflag = CS8 | PARENB | CSTOPB,
    .c_speed = 100000
  };

  assert(ioctl_termios(TCSETS, &requested) == 0);
  assert(g_regs[STM32_USART_BRR_OFFSET / 4] == 640);
  assert((g_regs[0] & (USART_CR1_PCE | USART_CR1_M0)) ==
         (USART_CR1_PCE | USART_CR1_M0));
  assert((g_regs[1] & USART_CR2_STOP_MASK) == USART_CR2_STOP2);
#ifdef CONFIG_STM32_USART_INVERT
  for (unsigned int invert = 0; invert < 4; invert++)
    {
      g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
      stm32serial_rxint(&g_priv.dev, true);
      uint16_t ie = g_priv.ie;

      assert(ioctl_arg(TIOCSINVERT, invert) == 0);
      assert(((g_regs[1] & USART_CR2_RXINV) != 0) == ((invert & 1) != 0));
      assert(((g_regs[1] & USART_CR2_TXINV) != 0) == ((invert & 2) != 0));
      assert((g_regs[1] & (1 << 18)) == 0);
      assert(g_priv.ie == ie && g_regs[3] == 640);
    }

  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  memcpy(before, g_regs, sizeof(before));
  assert(ioctl_arg(TIOCSINVERT, 4) == -EINVAL);
  assert(memcmp(before, g_regs, sizeof(before)) == 0);
  g_enable_failure = 1;
  assert(ioctl_arg(TIOCSINVERT, 0) == -ETIMEDOUT);
  assert(memcmp(before, g_regs, 4 * sizeof(uint32_t)) == 0);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  assert(ioctl_arg(TIOCSINVERT, 0) == 0);
#endif
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  requested.c_cflag = CS8;
  requested.c_speed = 115200;
  assert(ioctl_termios(TCSETS, &requested) == 0);
  assert(g_regs[3] == 556 && (g_regs[0] & USART_CR1_PCE) == 0);

#ifdef CONFIG_STM32_USART_SINGLEWIRE
#ifdef CONFIG_STM32_USART_INVERT
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  assert(ioctl_arg(TIOCSINVERT,
                   SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX) == 0);
#endif
  for (unsigned int pull = 0; pull <= 2; pull++)
    {
      for (unsigned int pushpull = 0; pushpull <= 1; pushpull++)
        {
          unsigned long mode = SER_SINGLEWIRE_ENABLED | (pull << 1) |
                               (pushpull ? SER_SINGLEWIRE_PUSHPULL : 0);

          g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
          assert(ioctl_arg(TIOCSSINGLEWIRE, mode) == 0);
          assert(g_regs[2] & USART_CR3_HDSEL);
          assert(((g_gpio[10] & GPIO_OPENDRAIN) == 0) == (pushpull != 0));
          assert((g_gpio[10] & GPIO_PUPD_MASK) ==
                 (pull == 1 ? GPIO_PULLUP :
                  pull == 2 ? GPIO_PULLDOWN : GPIO_FLOAT));
          assert(g_gpio[11] == g_config.rx_gpio);
        }
    }

  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  assert(ioctl_termios(TCSETS, &requested) == 0);
  assert(g_regs[2] & USART_CR3_HDSEL);
#ifdef CONFIG_STM32_USART_INVERT
  assert((g_regs[1] & (USART_CR2_RXINV | USART_CR2_TXINV)) ==
         (USART_CR2_RXINV | USART_CR2_TXINV));
#endif
  requested.c_cflag |= CRTSCTS;
  assert(ioctl_termios(TCSETS, &requested) == -EINVAL);
  requested.c_cflag = CS8;
  assert(ioctl_arg(TIOCSSINGLEWIRE,
                   SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PULL_MASK) ==
         -EINVAL);
  assert(ioctl_arg(TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED | 16) == -EINVAL);
  g_gpio_failure = 1;
  uint32_t gpio = g_priv.sw_gpio;

  memcpy(before, g_regs, sizeof(before));
  assert(ioctl_arg(TIOCSSINGLEWIRE, 0) == -EIO);
  assert(g_gpio[10] == gpio && g_priv.sw_gpio == gpio);
  assert(memcmp(before, g_regs, 4 * sizeof(uint32_t)) == 0);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  g_enable_failure = 1;
  assert(ioctl_arg(TIOCSSINGLEWIRE, 0) == -ETIMEDOUT);
  assert(g_gpio[10] == gpio && g_priv.sw_gpio == gpio);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  assert(ioctl_arg(TIOCSSINGLEWIRE,
                   ~(unsigned long)SER_SINGLEWIRE_ENABLED) == 0);
  assert((g_regs[2] & USART_CR3_HDSEL) == 0);
  assert(g_gpio[10] == g_config.tx_gpio);

  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || \
    defined(CONFIG_SERIAL_OFLOWCONTROL)
  requested.c_cflag = CS8;
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  requested.c_cflag |= CRTS_IFLOW;
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  requested.c_cflag |= CCTS_OFLOW;
#endif
  assert(ioctl_termios(TCSETS, &requested) == 0);
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  assert(ioctl_arg(TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED) == -EINVAL);
  requested.c_cflag = CS8;
  assert(ioctl_termios(TCSETS, &requested) == 0);
#endif
  g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_TC;
  unsigned int delays = g_delays;

  assert(ioctl_arg(TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED) == -EBUSY);
  assert(g_delays == delays);
#ifdef STM32_SERIAL_TXDMA
  g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
  g_priv.txdma_state = STM32_SERIAL_TXDMA_BATCH;
  assert(ioctl_arg(TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED) == -EBUSY);
  g_priv.txdma_state = STM32_SERIAL_TXDMA_IDLE;
#endif
#endif

  reset();
  configure();
  g_enable_failure = 2;
#ifdef CONFIG_STM32_USART_INVERT
  assert(ioctl_arg(TIOCSINVERT, SER_INVERT_ENABLED_RX) == -EIO);
#else
  assert(ioctl_arg(TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED) == -EIO);
#endif
  assert(g_errors != 0 && g_delays == 2 * USART_ACK_TIMEOUT_US);
#endif
#endif
}

#ifdef STM32_SERIAL_TXDMA
static void dma_reset(void)
{
  reset();
  g_regs[STM32_USART_CR1_OFFSET / 4] =
    USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void dma_complete(size_t count, uint8_t status)
{
  const uint8_t *source = (const uint8_t *)g_dma_config.source_address;
  size_t i;

  assert(count <= g_dma_config.nbytes);
  for (i = 0; i < count; i++)
    {
      g_txbytes[g_txcount++] = source[i] & (g_priv.bits == 7 ? 0x7f : 0xff);
    }
  g_dma_transferred = count;
  if ((status & DMA_STATUS_DTEF) != 0)
    {
      g_dma_progress_result = -EIO;
    }

  stm32serial_dmatxcallback(g_priv.txdma, status, &g_priv);
}

static void test_dma(void)
{
  static char large[131073];
  static const unsigned int lengths[] =
    {1, 2, 15, 16, 17, 65534, 65535, 65536, 131071};
  unsigned int i;
  unsigned int j;

  for (i = 0; i < sizeof(large); i++)
    {
      large[i] = i * 13;
    }

  for (i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++)
    {
      dma_reset();
      stm32serial_dmainitialize(&g_priv);
      g_priv.dev.xmit.buffer = large;
      g_priv.dev.xmit.size = sizeof(large);
      g_priv.dev.xmit.head = lengths[i];
      stm32serial_dmatxavail(&g_priv.dev);
      while (g_priv.txdma_state != STM32_SERIAL_TXDMA_IDLE)
        {
          assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BLOCK);
          assert(!stm32serial_txempty(&g_priv.dev));
          dma_complete(g_dma_config.nbytes, DMA_STATUS_TCF);
        }

      assert(g_priv.dev.xmit.tail == lengths[i]);
      assert(g_txcount == lengths[i]);
      assert(memcmp(g_txbytes, large, lengths[i]) == 0);
      g_regs[STM32_USART_ISR_OFFSET / 4] &= ~USART_ISR_TC;
      assert(!stm32serial_txempty(&g_priv.dev));
      g_regs[STM32_USART_ISR_OFFSET / 4] |= USART_ISR_TC;
      assert(stm32serial_txempty(&g_priv.dev));
    }

  g_priv.dev.xmit.buffer = g_txbuffer;
  g_priv.dev.xmit.size = sizeof(g_txbuffer);
  for (i = 0; i < sizeof(g_txbuffer); i++)
    {
      g_txbuffer[i] = i * 7;
    }

  dma_reset();
  stm32serial_dmainitialize(&g_priv);
  g_priv.dev.xmit.tail = 500;
  g_priv.dev.xmit.head = 10;
  stm32serial_dmatxavail(&g_priv.dev);
  dma_complete(12, DMA_STATUS_TCF);
  dma_complete(4, DMA_STATUS_SUSPF);
  assert(g_priv.txdma_fallback &&
         g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);
  assert(g_priv.dev.xmit.tail == 10 && g_txcount == 22);
  for (i = 0; i < 22; i++)
    {
      assert(g_txbytes[i] == (uint8_t)g_txbuffer[(500 + i) % 512]);
    }

  for (j = 0; j < 2; j++)
    {
      dma_reset();
      stm32serial_dmainitialize(&g_priv);
      g_priv.dev.xmit.tail = 500;
      g_priv.dev.xmit.head = 10;
      stm32serial_dmatxavail(&g_priv.dev);
      if (j == 0)
        {
          g_dma_setup_result = -EINVAL;
        }
      else
        {
          g_dma_start_result = -EIO;
        }

      dma_complete(12, DMA_STATUS_TCF);
      assert(g_priv.dev.xmit.tail == 10 && g_txcount == 22);
      for (i = 0; i < 22; i++)
        {
          assert(g_txbytes[i] == (uint8_t)g_txbuffer[(500 + i) % 512]);
        }
    }

  for (j = 0; j < 2; j++)
    {
      dma_reset();
      stm32serial_dmainitialize(&g_priv);
      g_priv.dev.xmit.head = 17;
      g_dma_setup_result = j == 0 ? -EINVAL : 0;
      g_dma_start_result = j == 1 ? -EIO : 0;
      stm32serial_dmatxavail(&g_priv.dev);
      assert(g_priv.txdma_fallback &&
             g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);
      assert(g_txcount == 17 && g_priv.dev.xmit.tail == 17);
      assert(memcmp(g_txbytes, g_txbuffer, 17) == 0);
    }

  for (j = 0; j <= 17; j++)
    {
      dma_reset();
      stm32serial_dmainitialize(&g_priv);
      g_priv.dev.xmit.head = 17;
      stm32serial_dmatxavail(&g_priv.dev);
      stm32serial_dmatxcallback(NULL, DMA_STATUS_TCF, &g_priv);
      stm32serial_dmatxcallback(g_priv.txdma, DMA_STATUS_HTF, &g_priv);
      assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BLOCK &&
             g_priv.dev.xmit.tail == 0);
      dma_complete(j, DMA_STATUS_SUSPF);
      assert(g_priv.txdma_fallback &&
             g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);
      assert(g_txcount == 17 && g_priv.dev.xmit.tail == 17);
      assert(memcmp(g_txbytes, g_txbuffer, 17) == 0);
    }

  for (j = 0; j < 2; j++)
    {
      dma_reset();
      stm32serial_dmainitialize(&g_priv);
      assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);
      g_priv.dev.xmit.head = 17;
      g_dma_setup_result = j == 0 ? -EINVAL : 0;
      g_dma_start_result = j == 1 ? -EIO : 0;
      g_dma_stop_result = -ETIMEDOUT;
      stm32serial_dmatxavail(&g_priv.dev);
      assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BATCH);
      assert(g_priv.txdma_fallback && g_priv.txdma_error == -ETIMEDOUT);
      assert(g_priv.dev.xmit.tail == 0 && g_txcount == 0);
      stm32serial_txint(&g_priv.dev, true);
      stm32serial_dmatxavail(&g_priv.dev);
      assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BATCH);
      assert(g_priv.dev.xmit.tail == 0 && g_txcount == 0);
      g_dma_stop_result = 0;
      assert(ioctl_arg(TCFLSH, TCOFLUSH) == 0);
      assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);
      assert(g_priv.txdma_error == 0 && g_priv.dev.xmit.tail == 17);
    }

  dma_reset();
  stm32serial_dmainitialize(&g_priv);
  g_priv.dev.xmit.head = 17;
  stm32serial_dmatxavail(&g_priv.dev);
  g_dma_stop_result = -ETIMEDOUT;
  dma_complete(4, DMA_STATUS_SUSPF);
  assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BLOCK);
  assert(g_priv.dev.xmit.tail == 0 && g_txcount == 4);
  assert(ioctl_arg(TCFLSH, TCOFLUSH) == -ETIMEDOUT);
  stm32serial_txint(&g_priv.dev, true);
  stm32serial_dmatxavail(&g_priv.dev);
  assert(g_priv.dev.xmit.tail == 0 && g_txcount == 4);
  g_dma_stop_result = 0;
  assert(ioctl_arg(TCFLSH, TCOFLUSH) == 0);
  assert(g_priv.dev.xmit.tail == 17 &&
         g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);

  dma_reset();
  g_dma_callback_result = -EINVAL;
  stm32serial_dmainitialize(&g_priv);
  assert(g_priv.txdma == NULL && g_priv.txdma_fallback);
  g_dma_free_result = -EBUSY;
  stm32serial_dmainitialize(&g_priv);
  assert(g_priv.txdma != NULL && g_priv.txdma_error == -EBUSY);

  dma_reset();
  stm32serial_dmainitialize(&g_priv);
  g_priv.dev.xmit.head = 17;
  stm32serial_dmatxavail(&g_priv.dev);
  dma_complete(4, DMA_STATUS_DTEF);
  assert(g_priv.txdma_error == -EIO &&
         g_priv.txdma_state == STM32_SERIAL_TXDMA_BLOCK);
  assert(g_priv.dev.xmit.tail == 0 && g_txcount == 4);
  assert(ioctl_arg(TCFLSH, TCOFLUSH) == 0);
  assert(g_priv.dev.xmit.tail == 17 &&
         g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);

  dma_reset();
  stm32serial_dmainitialize(&g_priv);
  g_priv.dev.xmit.head = 17;
  stm32serial_dmatxavail(&g_priv.dev);
  assert((g_regs[STM32_USART_ISR_OFFSET / 4] & USART_ISR_TC) == 0);
  assert(ioctl_arg(TCFLSH, TCIOFLUSH) == 0);
  assert(g_priv.dev.xmit.tail == 17 && stm32serial_txempty(&g_priv.dev));
  assert(g_txcount == 0);

  dma_reset();
  stm32serial_dmainitialize(&g_priv);
  g_priv.dev.xmit.head = 17;
  stm32serial_dmatxavail(&g_priv.dev);
  up_putc('\n');
  up_putc('D');
  stm32serial_detach(&g_priv.dev);
  assert(g_irq_detaches == 0);
  assert(g_txcount == 0);
  g_priv.dev.xmit.head = 20;
  dma_complete(17, DMA_STATUS_TCF);
  assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_IDLE);
  assert(stm32serial_debugsend(&g_priv));
  assert(g_txcount == 20 && g_txbytes[17] == '\r' &&
         g_txbytes[18] == '\n' && g_txbytes[19] == 'D');
  assert(g_priv.txdma_state == STM32_SERIAL_TXDMA_BLOCK);
  dma_complete(3, DMA_STATUS_TCF);
  assert(g_txcount == 23 && g_priv.dev.xmit.tail == 20);

  dma_reset();
  stm32serial_dmainitialize(&g_priv);
  g_priv.dev.xmit.head = 1;
  stm32serial_dmatxavail(&g_priv.dev);
  for (i = 0; i < USART_DEBUG_BUFSIZE + 10; i++)
    {
      up_putc('!');
    }

  assert(g_debugoverflow);
  dma_complete(1, DMA_STATUS_TCF);
  while (g_debughead != g_debugtail)
    {
      assert(stm32serial_debugsend(&g_priv));
    }

  assert(!g_debugoverflow);
  assert(g_txcount == 1 + USART_DEBUG_BUFSIZE - 1 +
         strlen("\r\nERROR: USART debug TX overflow\r\n"));

  dma_reset();
  g_dma_available = false;
  stm32serial_dmainitialize(&g_priv);
  g_priv.dev.xmit.head = 17;
  stm32serial_dmatxavail(&g_priv.dev);
  assert(g_priv.txdma == NULL && g_txcount == 17);

  dma_reset();
  stm32serial_dmainitialize(&g_priv);
  g_priv.bits = 7;
  g_priv.dev.xmit.head = 17;
  stm32serial_dmatxavail(&g_priv.dev);
  assert(!g_priv.txdma_fallback &&
         g_priv.txdma_state == STM32_SERIAL_TXDMA_BLOCK);
  dma_complete(17, DMA_STATUS_TCF);
  assert(g_txcount == 17);
  for (i = 0; i < 17; i++)
    {
      assert(g_txbytes[i] == ((uint8_t)g_txbuffer[i] & 0x7f));
    }

  for (i = 0; i < 8; i++)
    {
      dma_reset();
      assert(stm32serial_setup(&g_priv.dev) == 0);
      stm32serial_dmainitialize(&g_priv);
      assert(g_priv.txdma != NULL && !g_priv.txdma_fallback);
      stm32serial_shutdown(&g_priv.dev);
      assert(g_priv.txdma == NULL && !g_priv.initialized);
      assert(g_priv.shutdown_error == 0);
    }
}
#endif

int main(void)
{
  reset();
  stm32serial_txint(&g_priv.dev, false);
  stm32serial_detach(&g_priv.dev);
  test_handoff();
  test_formats();
  test_busy_and_rollback();
  test_fifo_and_debug();
  test_pm_and_close();
  test_initial_flow();
  test_flow();
  test_rc_modes();
#ifdef STM32_SERIAL_TXDMA
  test_dma();
#endif
  assert(g_priv.lock == 0 && g_lock_depth == 0);
  return 0;
}
