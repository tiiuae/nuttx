/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_i2c_driver_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define CONFIG_STM32N6_STM32N6XXXX 1
#define CONFIG_STM32N6_I2C1 1
#define CONFIG_STM32N6_I2C2 1
#define CONFIG_STM32N6_I2C3 1
#define CONFIG_STM32N6_I2C4 1
#define CONFIG_I2C_POLLED 1
#define CONFIG_STM32N6_I2CTIMEOTICKS 2000
#define STM32_HSI_FREQUENCY 64000000
#define GPIO_I2C1_SCL 1
#define GPIO_I2C1_SDA 2
#define GPIO_I2C2_SCL 3
#define GPIO_I2C2_SDA 4
#define GPIO_I2C3_SCL 5
#define GPIO_I2C3_SDA 6
#define GPIO_I2C4_SCL 7
#define GPIO_I2C4_SDA 8
#define DEBUGASSERT(c) assert(c)
#define UNUSED(v) (void)(v)
#define USEC_PER_SEC 1000000
#define USEC2TICK(v) ((v) / 1000)
#define i2cinfo(...) ((void)0)
#define i2cwarn(...) ((void)0)
#define i2cerr(...) ((void)0)
#define stm32_i2c_tracereset(p) ((void)0)
#define stm32_i2c_tracenew(p, s) ((void)0)
#define stm32_i2c_traceevent(p, e, a) ((void)0)
#define stm32_i2c_tracedump(p) ((void)0)
#define I2C_INT_BAD_STATE 0x8000000
#define OK 0
#define ERROR (-1)
#define NXMUTEX_INITIALIZER 0

#include <nuttx/i2c/i2c_master.h>
#include "../../hardware/stm32n6xxx_i2c.h"
#include "../../hardware/stm32n6xxx_rcc.h"

typedef int mutex_t;
typedef int irqstate_t;

struct i2c_master_s
{
  const void *ops;
};

/* DRIVER_TYPES */
/* DRIVER_CONFIGS */

static uint32_t registers[11];
static uint32_t hsicfgr;
static uint32_t ccipr4;
static uintptr_t last_rcc_address;
static uint32_t last_rcc_value;
static uint32_t injected_status;
static unsigned int bytes;
static unsigned int starts;
static unsigned int stops;
static unsigned int nbytes;
static unsigned int timing_writes;
static unsigned int peripheral_disables;
static uint32_t start_cr2[4];
static clock_t ticks;
static bool stalled;
static bool locked;

static int nxmutex_lock(mutex_t *lock)
{
  (void)lock;
  assert(!locked);
  locked = true;
  return OK;
}

static void nxmutex_unlock(mutex_t *lock)
{
  (void)lock;
  assert(locked);
  locked = false;
}

static irqstate_t enter_critical_section(void)
{
  return 0;
}

static void leave_critical_section(irqstate_t flags)
{
  (void)flags;
}

static clock_t clock_systime_ticks(void)
{
  return ticks++;
}

static int stm32_configgpio(uint32_t pin)
{
  (void)pin;
  return OK;
}

static void stm32_unconfiggpio(uint32_t pin)
{
  (void)pin;
}

static void update_status(void)
{
  uint32_t cr2 = registers[STM32_I2C_CR2_OFFSET / 4];

  registers[STM32_I2C_ISR_OFFSET / 4] = stalled ? 0 :
      injected_status != 0 ? injected_status :
      nbytes != 0 ? ((cr2 & I2C_CR2_RD_WRN) ?
                    I2C_ISR_RXNE : I2C_ISR_TXIS) :
      (cr2 & I2C_CR2_RELOAD) ? I2C_ISR_TCR : I2C_ISR_TC;
}

static uint32_t getreg32(uintptr_t address)
{
  if (address == STM32_RCC_HSICFGR)
    {
      return hsicfgr;
    }

  if (address == STM32_RCC_CCIPR4)
    {
      return ccipr4;
    }

  assert(address >= STM32_I2C2_BASE &&
         address <= STM32_I2C2_BASE + STM32_I2C_TXDR_OFFSET);
  if (address == STM32_I2C2_BASE + STM32_I2C_RXDR_OFFSET)
    {
      bytes++;
      assert(nbytes > 0);
      nbytes--;
      update_status();
      return bytes & 0xff;
    }

  return registers[(address - STM32_I2C2_BASE) / 4];
}

static uint16_t getreg16(uintptr_t address)
{
  return getreg32(address);
}

static void putreg32(uint32_t value, uintptr_t address)
{
  uint32_t old;
  unsigned int index;

  if (address == STM32_RCC_CCIPR4)
    {
      ccipr4 = value;
      return;
    }

  if (address < STM32_I2C2_BASE ||
      address > STM32_I2C2_BASE + STM32_I2C_TXDR_OFFSET)
    {
      last_rcc_address = address;
      last_rcc_value = value;
      return;
    }

  index = (address - STM32_I2C2_BASE) / 4;
  old = registers[index];
  registers[index] = value;
  if (address == STM32_I2C2_BASE + STM32_I2C_TIMINGR_OFFSET)
    {
      timing_writes++;
    }
  else if (address == STM32_I2C2_BASE + STM32_I2C_CR1_OFFSET)
    {
      if ((old & I2C_CR1_PE) != 0 && (value & I2C_CR1_PE) == 0)
        {
          peripheral_disables++;
        }
    }
  else if (address == STM32_I2C2_BASE + STM32_I2C_TXDR_OFFSET)
    {
      bytes++;
      assert(nbytes > 0);
      nbytes--;
      update_status();
    }
  else if (address == STM32_I2C2_BASE + STM32_I2C_CR2_OFFSET)
    {
      if (value & I2C_CR2_START)
        {
          assert(starts < 4);
          start_cr2[starts++] = value;
          nbytes = (value & I2C_CR2_NBYTES_MASK) >> I2C_CR2_NBYTES_SHIFT;
          registers[index] &= ~I2C_CR2_START;
          update_status();
        }
      else if (value & I2C_CR2_STOP)
        {
          stops++;
          registers[index] &= ~I2C_CR2_STOP;
          registers[STM32_I2C_ISR_OFFSET / 4] = I2C_ISR_TXE;
        }
      else if ((old ^ value) & I2C_CR2_NBYTES_MASK ||
               registers[STM32_I2C_ISR_OFFSET / 4] == I2C_ISR_TCR)
        {
          nbytes = (value & I2C_CR2_NBYTES_MASK) >> I2C_CR2_NBYTES_SHIFT;
          update_status();
        }
    }
  else if (address == STM32_I2C2_BASE + STM32_I2C_ICR_OFFSET)
    {
      registers[STM32_I2C_ISR_OFFSET / 4] &= ~value;
    }
}

static void putreg16(uint16_t value, uintptr_t address)
{
  putreg32(value, address);
}

static void modifyreg32(uintptr_t address, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(address) & ~clear) | set, address);
}

static int stm32_i2c_sem_waitdone(struct stm32_i2c_priv_s *priv);
static int stm32_i2c_deinit(struct stm32_i2c_priv_s *priv);

/* DRIVER_ROUTINES */

static void reset_mock(void)
{
  memset(registers, 0, sizeof(registers));
  bytes = starts = stops = nbytes = 0;
  timing_writes = peripheral_disables = 0;
  injected_status = 0;
  stalled = false;
  ticks = 0;
  locked = false;
}

static void test_timing(void)
{
  const uint32_t frequencies[] = {100000, 400000, 1000000, 10000};
  const uint32_t expected[] =
  {
    0x30504359, 0x30300c18, 0x30200507, 0xf01097f7
  };
  const uint32_t low_min_ns[] = {4700, 1300, 500, 4700};
  const uint32_t high_min_ns[] = {4000, 600, 260, 4000};
  const uint32_t setup_min_ns[] = {250, 100, 50, 250};
  const uint32_t valid_max_ns[] = {3450, 900, 450, 3450};
  const uint64_t kernel_period_ps = 15625;
  unsigned int divider;
  unsigned int i;

  hsicfgr = 0;
  for (i = 0; i < 4; i++)
    {
      uint32_t timing;
      uint32_t presc;
      uint32_t sclh;
      uint32_t scll;
      uint32_t scldel;
      uint32_t sdadel;
      uint64_t tpresc;
      uint64_t period;

      reset_mock();
      stm32_i2c2_priv.frequency = 0;
      registers[STM32_I2C_CR1_OFFSET / 4] =
          I2C_CR1_PE | I2C_CR1_ANFOFF | I2C_CR1_DNF_MASK;
      assert(stm32_i2c_setclock(&stm32_i2c2_priv, frequencies[i]) == OK);
      timing = registers[STM32_I2C_TIMINGR_OFFSET / 4];
      assert(timing == expected[i]);
      presc = timing >> I2C_TIMINGR_PRESC_SHIFT;
      sclh = (timing & I2C_TIMINGR_SCLH_MASK) >> I2C_TIMINGR_SCLH_SHIFT;
      scll = timing & I2C_TIMINGR_SCLL_MASK;
      scldel = (timing & I2C_TIMINGR_SCLDEL_MASK) >> I2C_TIMINGR_SCLDEL_SHIFT;
      sdadel = (timing & I2C_TIMINGR_SDADEL_MASK) >> I2C_TIMINGR_SDADEL_SHIFT;
      tpresc = (presc + 1) * kernel_period_ps;

      /* Minimum filter/synchronizer delay, without relying on rise/fall
       * times to slow SCL. Check setup/valid times at the stated envelope.
       */

      period = (sclh + scll + 2) * tpresc + 100000 + 4 * kernel_period_ps;
      assert(period * frequencies[i] >= 1000000000000ull);
      assert((scll + 1) * tpresc >= low_min_ns[i] * 1000ull);
      assert((sclh + 1) * tpresc >= high_min_ns[i] * 1000ull);
      assert((scldel + 1) * tpresc >= (120 + setup_min_ns[i]) * 1000ull);
      assert(sdadel * tpresc + 120000 + 260000 + 4 * kernel_period_ps <=
             valid_max_ns[i] * 1000ull);
      assert(50000 + 3 * kernel_period_ps >= 10000);
      assert(stm32_i2c2_priv.frequency == frequencies[i]);
      assert(registers[STM32_I2C_CR1_OFFSET / 4] & I2C_CR1_PE);
      assert(!(registers[STM32_I2C_CR1_OFFSET / 4] &
               (I2C_CR1_ANFOFF | I2C_CR1_DNF_MASK)));
      assert(!!(registers[STM32_I2C_CR1_OFFSET / 4] & I2C_CR1_FMP) ==
             (frequencies[i] == 1000000));
      assert(peripheral_disables == 1 && timing_writes == 1);
      assert(stm32_i2c_setclock(&stm32_i2c2_priv, frequencies[i]) == OK);
      assert(peripheral_disables == 2 && timing_writes == 1);
    }

  reset_mock();
  hsicfgr = 0;
  stm32_i2c2_priv.frequency = 0;
  assert(stm32_i2c_setclock(&stm32_i2c2_priv, 0) == -EINVAL);
  assert(registers[STM32_I2C_TIMINGR_OFFSET / 4] == 0);
  assert(stm32_i2c_setclock(&stm32_i2c2_priv, 200000) == OK);
  assert(registers[STM32_I2C_TIMINGR_OFFSET / 4] == expected[3]);
  assert(stm32_i2c2_priv.frequency == 10000);

  for (divider = 1; divider < 4; divider++)
    {
      reset_mock();
      hsicfgr = divider << RCC_HSICFGR_HSIDIV_SHIFT;
      assert(stm32_i2c_setclock(&stm32_i2c2_priv, 100000) == -ENOTSUP);
      assert(timing_writes == 0 && peripheral_disables == 0);
    }
}

static void test_transfers(void)
{
  struct stm32_i2c_inst_s inst = {NULL, &stm32_i2c2_priv};
  uint8_t buffer[300] = {0};
  struct i2c_msg_s msgs[2] =
  {
    {100000, 0x76, I2C_M_NOSTOP, buffer, 1},
    {100000, 0x76, I2C_M_READ, buffer + 1, 2}
  };

  hsicfgr = 0;
  reset_mock();
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 2) == OK);
  assert(bytes == 3 && starts == 2 && stops == 1 && !locked);
  assert((start_cr2[0] & I2C_CR2_SADD7_MASK) == (0x76 << 1));
  assert(start_cr2[1] & I2C_CR2_RD_WRN);
  assert(buffer[1] == 2 && buffer[2] == 3);

  reset_mock();
  msgs[0].length = sizeof(buffer);
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 1) == OK);
  assert(bytes == sizeof(buffer) && starts == 1 && stops == 1);

  reset_mock();
  msgs[0].flags = I2C_M_READ;
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 1) == OK);
  assert(bytes == sizeof(buffer) && starts == 1 && stops == 1);
  assert(buffer[255] == 0 && buffer[299] == 44);

  reset_mock();
  msgs[0].length = 1;
  msgs[0].flags = 0;
  msgs[1].flags = I2C_M_NOSTART;
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 2) == OK);
  assert(bytes == 3 && starts == 1 && stops == 1);

  reset_mock();
  injected_status = I2C_ISR_NACKF;
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 1) ==
         -EADDRNOTAVAIL);
  assert(bytes == 0 && !locked);

  reset_mock();
  injected_status = I2C_ISR_BERR;
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 1) == -EIO);

  reset_mock();
  stalled = true;
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 1) ==
         -ETIMEDOUT);
  assert(!locked);

  reset_mock();
  msgs[0].frequency = 0;
  assert(stm32_i2c_transfer((struct i2c_master_s *)&inst, msgs, 1) == -EINVAL);
  assert(starts == 0 && !locked);
}

int main(void)
{
  assert(stm32_i2c1_priv.config->clk_set == STM32_RCC_APB1LENSR);
  assert(stm32_i2c3_priv.config->reset_set == STM32_RCC_APB1LRSTSR);
  assert(stm32_i2c4_priv.config->clk_set == STM32_RCC_APB4LENSR);
  assert(stm32_i2c4_priv.config->reset_clear == STM32_RCC_APB4LRSTCR);
  assert(stm32_i2c_init(&stm32_i2c2_priv) == OK);
  assert(last_rcc_address == STM32_RCC_APB1LRSTCR);
  assert(last_rcc_value == RCC_APB1LRSTR_I2C2RST);
  assert((ccipr4 & RCC_CCIPR4_I2C2SEL_MASK) ==
         RCC_CCIPR4_I2C2SEL_HSI_DIV_CK);
  assert(stm32_i2c_deinit(&stm32_i2c2_priv) == OK);
  assert(last_rcc_address == STM32_RCC_APB1LENCR);
  test_timing();
  test_transfers();
  puts("STM32N6 adapted I2C driver tests passed");
  return 0;
}
