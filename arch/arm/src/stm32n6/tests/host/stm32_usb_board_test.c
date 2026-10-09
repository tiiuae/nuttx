/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_usb_board_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OK 0
#define NXMUTEX_INITIALIZER 0
#define I2C_SPEED_STANDARD 100000
#define I2C_M_READ 1
#define LPWORK 1
#define MSEC2TICK(n) (n)
#define CONTROL_NPRIV 1
#define GPIO_OUTPUT 0x100
#define GPIO_INPUT 0x200
#define GPIO_PUSHPULL 0x400
#define GPIO_SPEED_2MHZ 0x800
#define GPIO_OUTPUT_CLEAR 0
#define GPIO_FLOAT 0
#define GPIO_PORTA 0x1000
#define GPIO_PORTD 0x4000
#define GPIO_PIN7 7
#define GPIO_PIN2 2
#define LOG_ERR 3
#define LOG_WARNING 4
#define LOG_INFO 6
#define syslog test_log

/* BOARD_HEADERS */

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uint32_t clock_t;
typedef int mutex_t;

struct i2c_master_s
{
  int unused;
};

struct i2c_msg_s
{
  uint32_t frequency;
  uint16_t addr;
  uint16_t flags;
  uint8_t *buffer;
  int length;
};

struct usbdev_s
{
  int unused;
};

struct work_s
{
  int unused;
};

static struct i2c_master_s g_bus;
static uint32_t g_clock;
static uint32_t g_sleep_clock;
static uint32_t g_reset;
static uint32_t g_hsi;
static uint32_t g_cfg;
static uint32_t g_cr;
static uint32_t g_sr;
static uint32_t g_tick;
static uint32_t g_control;
static uint32_t g_due;
static uint8_t g_ack;
static uint8_t g_flags;
static bool g_flag_pin;
static bool g_enable;
static bool g_present;
static bool g_irq_context;
static unsigned int g_bindings;
static unsigned int g_bus_inits;
static unsigned int g_registrations;
static unsigned int g_bus_uninits;
static unsigned int g_commands;
static unsigned int g_register_writes;
static unsigned int g_gpio_calls;
static unsigned int g_attach;
static unsigned int g_detach;
static unsigned int g_logs;
static int g_bind_result;
static int g_bus_result;
static int g_register_result;
static int g_uninit_result;
static int g_transfer_result;
static int g_queue_result;
static int g_vbus_result;
static int g_gpio_result;
static int g_sleep_result;
static int g_monitor_result;
static uintptr_t g_reject_register;
static void (*g_worker)(void *);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void test_log(int priority, const char *format, ...)
{
  (void)priority;
  (void)format;
  g_logs++;
}

static int nxmutex_lock(mutex_t *lock)
{
  assert(*lock == 0);
  *lock = 1;
  return OK;
}

static int nxmutex_unlock(mutex_t *lock)
{
  assert(*lock == 1);
  *lock = 0;
  return OK;
}

static bool up_interrupt_context(void)
{
  return g_irq_context;
}

static uint32_t getcontrol(void)
{
  return g_control;
}

static uint32_t getreg32(uintptr_t reg)
{
  switch (reg)
    {
      case STM32_RCC_SR:
        return g_hsi;
      case STM32_RCC_APB1HENR:
        return g_clock;
      case STM32_RCC_APB1HLPENR:
        return g_sleep_clock;
      case STM32_RCC_APB1HRSTR:
        return g_reset;
      case STM32_UCPD_CFGR1:
        return g_cfg;
      case STM32_UCPD_CR:
        return g_cr;
      case STM32_UCPD_SR:
        return g_sr;
      default:
        assert(false);
        return 0;
    }
}

static void putreg32(uint32_t value, uintptr_t reg)
{
  g_register_writes++;
  if (reg == g_reject_register)
    {
      return;
    }

  switch (reg)
    {
      case STM32_RCC_APB1HENSR:
        assert(value == RCC_APB1HENR_UCPD1EN);
        g_clock |= value;
        break;
      case STM32_RCC_APB1HENCR:
        assert(value == RCC_APB1HENR_UCPD1EN);
        g_clock &= ~value;
        break;
      case STM32_RCC_APB1HLPENSR:
        assert(value == RCC_APB1HLPENR_UCPD1LPEN);
        g_sleep_clock |= value;
        break;
      case STM32_RCC_APB1HLPENCR:
        assert(value == RCC_APB1HLPENR_UCPD1LPEN);
        g_sleep_clock &= ~value;
        break;
      case STM32_RCC_APB1HRSTSR:
        assert(value == RCC_APB1HRSTR_UCPD1RST);
        g_reset |= value;
        g_cfg = 0;
        g_cr = 0;
        break;
      case STM32_RCC_APB1HRSTCR:
        assert(value == RCC_APB1HRSTR_UCPD1RST);
        g_reset &= ~value;
        break;
      case STM32_UCPD_CFGR1:
        g_cfg = value;
        break;
      case STM32_UCPD_CR:
        assert(value == (UCPD_CR_ANAMODE_SINK | UCPD_CR_CCENABLE_BOTH));
        g_cr = value;
        break;
      case STM32_UCPD_CFGR2:
      case STM32_UCPD_IMR:
        assert(value == 0);
        break;
      default:
        assert(false);
    }
}

static int stm32_configgpio(uint32_t gpio)
{
  g_gpio_calls++;
  assert(gpio == (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ |
                  GPIO_PORTA | GPIO_PIN7) ||
         gpio == (GPIO_INPUT | GPIO_PORTD | GPIO_PIN2));
  return g_gpio_result;
}

static void stm32_gpiowrite(uint32_t gpio, bool value)
{
  assert((gpio & 0xff) == GPIO_PIN7);
  g_enable = value;
}

static bool stm32_gpioread(uint32_t gpio)
{
  assert((gpio & 0xff) == GPIO_PIN2);
  return g_flag_pin;
}

static clock_t clock_systime_ticks(void)
{
  return g_tick;
}

static int nxsig_usleep(unsigned int usec)
{
  assert(usec == 2000);
  return g_sleep_result;
}

static int work_queue(int queue, struct work_s *work,
                      void (*worker)(void *), void *arg, clock_t delay)
{
  assert(queue == LPWORK && work != NULL && arg == NULL);
  if (g_queue_result < 0)
    {
      return g_queue_result;
    }

  assert(g_worker == NULL);
  g_worker = worker;
  g_due = g_tick + delay;
  return OK;
}

static int cdcacm_initialize(int minor, void *handle)
{
  assert(minor == 0 && handle == NULL);
  g_bindings++;
  return g_bind_result;
}

static int usbmonitor_start(void)
{
  return g_monitor_result;
}

static int stm32_usbdev_vbus(bool present)
{
  if (g_vbus_result < 0)
    {
      return g_vbus_result;
    }

  if (g_present != present)
    {
      if (present)
        {
          g_attach++;
        }
      else
        {
          g_detach++;
        }
    }

  g_present = present;
  return OK;
}

static int test_transfer(struct i2c_master_s *bus,
                          struct i2c_msg_s *msgs, int count)
{
  assert(bus == &g_bus);
  assert(msgs[0].frequency == 100000 && msgs[0].addr == 0x34);
  assert(msgs[0].flags == 0);
  if (g_transfer_result < 0)
    {
      return g_transfer_result;
    }

  if (count == 1)
    {
      assert(msgs[0].length == 2 && msgs[0].buffer[0] == 0);
      assert(msgs[0].buffer[1] == 0x28);
      g_commands++;
    }
  else
    {
      assert(count == 2 && msgs[0].length == 1);
      assert(msgs[1].length == 1 && msgs[1].flags == I2C_M_READ);
      assert(msgs[1].frequency == 100000 && msgs[1].addr == 0x34);
      assert(msgs[0].buffer[0] == 1 || msgs[0].buffer[0] == 2);
      *msgs[1].buffer = msgs[0].buffer[0] == 1 ? g_ack : g_flags;
    }

  return OK;
}

#define I2C_TRANSFER(d, m, n) test_transfer(d, m, n)

struct i2c_master_s *stm32_i2cbus_initialize(int port)
{
  assert(port == 2);
  g_bus_inits++;
  return g_bus_result < 0 ? NULL : &g_bus;
}

int stm32_i2cbus_uninitialize(struct i2c_master_s *bus)
{
  assert(bus == &g_bus);
  g_bus_uninits++;
  return g_uninit_result;
}

int i2c_register(struct i2c_master_s *bus, int port)
{
  assert(bus == &g_bus && port == 2);
  g_registrations++;
  return g_register_result;
}

/* BOARD_I2C */

/* BOARD_POLICY */

static void test_reset(void)
{
  memset(&g_cn8, 0, sizeof(g_cn8));
  g_i2c2 = NULL;
  g_usb_lock = 0;
  g_i2c_lock = 0;
  g_clock = 1u << 6;
  g_sleep_clock = 1u << 6;
  g_reset = 0;
  g_hsi = RCC_SR_HSIRDY;
  g_cfg = 0;
  g_cr = 0;
  g_sr = 0;
  g_tick = 0;
  g_control = 0;
  g_ack = 0x18;
  g_flags = 0;
  g_flag_pin = true;
  g_enable = false;
  g_present = false;
  g_irq_context = false;
  g_bindings = 0;
  g_bus_inits = 0;
  g_registrations = 0;
  g_bus_uninits = 0;
  g_commands = 0;
  g_register_writes = 0;
  g_gpio_calls = 0;
  g_attach = 0;
  g_detach = 0;
  g_logs = 0;
  g_bind_result = 0;
  g_bus_result = 0;
  g_register_result = 0;
  g_uninit_result = 0;
  g_transfer_result = 0;
  g_queue_result = 0;
  g_vbus_result = 0;
  g_gpio_result = 0;
  g_sleep_result = 0;
  g_monitor_result = 0;
  g_reject_register = 0;
  g_worker = NULL;
}

#ifdef CONFIG_NUCLEO_N657X0_Q_USBDEV_QUALIFIED
static void test_poll(void)
{
  void (*worker)(void *) = g_worker;

  assert(worker != NULL);
  g_tick = g_due;
  g_worker = NULL;
  worker(NULL);
}

static void test_cable(unsigned int cc)
{
  assert(cc <= 2);
  g_flags = cc == 0 ? 0 : TCPP_VBUS_OK;
  g_flag_pin = cc == 0;
  g_sr = cc == 0 ? 0 :
         1u << (cc == 1 ? UCPD_SR_CC1_SHIFT : UCPD_SR_CC2_SHIFT);
}

static void test_attach(unsigned int cc)
{
  unsigned int count;

  test_cable(cc);
  test_poll();
  for (count = 0; count < 7; count++)
    {
      test_poll();
      assert(!g_present);
    }

  test_poll();
  assert(g_present);
}

static void test_lifecycle(void)
{
  unsigned int cc;

  for (cc = 1; cc <= 2; cc++)
    {
      test_reset();
      assert(nucleo_usbdev_initialize() == OK);
      assert(nucleo_usbdev_initialize() == OK);
      assert(g_bindings == 1 && g_bus_inits == 1 && g_commands == 1);
      assert(g_enable && !g_present);
      test_attach(cc);
      assert(g_attach == 1);
      test_poll();
      assert(g_attach == 1);

      /* VBUS loss alone detaches, even while a CC state remains present. */

      g_flags = 0;
      test_poll();
      assert(!g_present && g_detach == 1);
      test_cable(0);
      test_poll();
      test_attach(cc == 1 ? 2 : 1);
      assert(g_attach == 2);
      g_sr = 0;
      test_poll();
      assert(!g_present && g_detach == 2);
    }

  test_reset();
  test_cable(1);
  assert(nucleo_usbdev_initialize() == OK);
  test_poll();
  assert(!g_present);
  test_cable(0);
  test_poll();
  test_attach(2);
  assert(g_attach == 1);

  test_reset();
  assert(nucleo_usbdev_initialize() == OK);
  test_cable(1);
  g_sr |= 1u << UCPD_SR_CC2_SHIFT;
  test_poll();
  assert(!g_present);
  test_cable(1);
  g_flag_pin = true;
  test_poll();
  assert(!g_present);

  test_reset();
  g_tick = UINT32_MAX - 60;
  assert(nucleo_usbdev_initialize() == OK);
  test_attach(1);
  assert(g_attach == 1);
}

static void test_faults(void)
{
  uintptr_t reject[] =
  {
    STM32_RCC_APB1HENSR, STM32_RCC_APB1HRSTSR,
    STM32_RCC_APB1HRSTCR, STM32_RCC_APB1HLPENSR,
    STM32_UCPD_CFGR1, STM32_UCPD_CR
  };

  unsigned int n;

  for (n = 0; n < sizeof(reject) / sizeof(reject[0]); n++)
    {
      test_reset();
      g_reject_register = reject[n];
      assert(nucleo_usbdev_initialize() < 0);
      assert(g_cn8.state == CN8_FAULT && !g_enable && !g_present);
      assert(g_worker == NULL && g_bindings == 1 && g_commands == 0);
      assert(g_clock == (1u << 6) && g_sleep_clock == (1u << 6));
      assert(nucleo_usbdev_initialize() < 0 && g_bindings == 1);
    }

  test_reset();
  g_hsi = 0;
  assert(nucleo_usbdev_initialize() == -EIO && !g_enable);
  test_reset();
  g_cfg = UCPD_CFGR1_UCPDEN;
  assert(nucleo_usbdev_initialize() == -EBUSY);
  assert(g_cfg == UCPD_CFGR1_UCPDEN && g_commands == 0);
  test_reset();
  g_bus_result = -ENODEV;
  assert(nucleo_usbdev_initialize() == -ENODEV && !g_enable);
  test_reset();
  g_ack = 0x14;
  assert(nucleo_usbdev_initialize() == -EIO && !g_enable);
  test_reset();
  g_flags = 0x80;
  assert(nucleo_usbdev_initialize() == -ENODEV && !g_enable);
  test_reset();
  g_transfer_result = -ETIMEDOUT;
  assert(nucleo_usbdev_initialize() == -ETIMEDOUT && !g_enable);
  test_reset();
  g_sleep_result = -EINTR;
  assert(nucleo_usbdev_initialize() == -EINTR && !g_enable);
  test_reset();
  g_queue_result = -ENOMEM;
  assert(nucleo_usbdev_initialize() == -ENOMEM && !g_enable);

  for (n = 0; n < 5; n++)
    {
      test_reset();
      assert(nucleo_usbdev_initialize() == OK);
      test_attach(1);
      g_flags |= 1u << n;
      test_poll();
      assert(g_cn8.state == CN8_FAULT && !g_present && !g_enable);
      assert(g_worker == NULL && g_detach == 1);
    }

  test_reset();
  assert(nucleo_usbdev_initialize() == OK);
  test_attach(2);
  g_transfer_result = -EIO;
  test_poll();
  assert(!g_present && !g_enable && g_worker == NULL);
  test_reset();
  assert(nucleo_usbdev_initialize() == OK);
  g_cr = 0;
  test_poll();
  assert(g_cn8.state == CN8_FAULT && !g_enable);
  test_reset();
  assert(nucleo_usbdev_initialize() == OK);
  g_queue_result = -EIO;
  test_poll();
  assert(g_cn8.state == CN8_FAULT && !g_enable);

  test_reset();
  g_clock |= RCC_APB1HENR_UCPD1EN;
  g_sleep_clock |= RCC_APB1HLPENR_UCPD1LPEN;
  assert(nucleo_usbdev_initialize() == OK);
  g_ack = 0;
  test_poll();
  assert((g_clock & RCC_APB1HENR_UCPD1EN) != 0);
  assert((g_sleep_clock & RCC_APB1HLPENR_UCPD1LPEN) != 0);
}
#endif

static void test_common(void)
{
  test_reset();
  assert(nucleo_i2c_initialize() == OK);
  assert(nucleo_i2c_initialize() == OK);
  assert(nucleo_i2c2_bus() == &g_bus && g_bus_inits == 1);
#ifdef CONFIG_I2C_DRIVER
  assert(g_registrations == 1);
  test_reset();
  g_register_result = -EIO;
  assert(nucleo_i2c_initialize() == -EIO);
  assert(g_bus_uninits == 1 && nucleo_i2c2_bus() == NULL);
#endif

  test_reset();
  g_control = CONTROL_NPRIV;
  assert(nucleo_usbdev_initialize() == -EACCES);
  assert(g_gpio_calls == 0 && g_register_writes == 0 && g_bindings == 0);
  test_reset();
  g_irq_context = true;
  assert(nucleo_usbdev_initialize() == -EWOULDBLOCK);
  assert(g_gpio_calls == 0 && g_register_writes == 0 && g_bindings == 0);
  test_reset();
  g_gpio_result = -EIO;
  assert(nucleo_usbdev_initialize() == -EIO && g_bindings == 0);
  test_reset();
  g_bind_result = -ENODEV;
  assert(nucleo_usbdev_initialize() == -ENODEV);
  assert(nucleo_usbdev_initialize() == -ENODEV && g_bindings == 1);
  test_reset();
  g_monitor_result = -ENOMEM;
  assert(nucleo_usbdev_initialize() == -ENOMEM && g_bindings == 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(void)
{
  test_common();
#ifdef CONFIG_NUCLEO_N657X0_Q_USBDEV_QUALIFIED
  test_lifecycle();
  test_faults();
#else
  test_reset();
  assert(nucleo_usbdev_initialize() == -EAGAIN);
  assert(nucleo_usbdev_initialize() == -EAGAIN);
  assert(g_cn8.state == CN8_BLOCKED && g_bindings == 1);
  assert(!g_enable && !g_present && g_worker == NULL);
  assert(g_register_writes == 0 && g_bus_inits == 0 && g_commands == 0);
#endif

  puts("CN8 static-sink policy: PASS");
  return 0;
}
