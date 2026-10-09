/****************************************************************************
 * arch/arm/src/stm32n6/tests/host/stm32_spi_driver_test.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_STM32_STM32N6XXXX
#define STM32_HSI_FREQUENCY 64000000u
#define STM32_CPUCLK_FREQUENCY 64000000u
#define DWT_CYCCNT 0x10000u
#define TEST_SPI_BASE 0x20000u
#define OK 0
#define spierr(...) ((void)0)

#include "hardware/stm32n6xxx_rcc.h"
#include "hardware/stm32n6xxx_spi.h"

typedef int mutex_t;

struct spi_dev_s
{
  int unused;
};

enum spi_mode_e
{
  SPIDEV_MODE0, SPIDEV_MODE1, SPIDEV_MODE2, SPIDEV_MODE3
};

/* DRIVER_DEFINITIONS */
/* DRIVER_TYPES */

static uint32_t g_regs[13];
static uint16_t g_fifo[8];
static size_t g_head;
static size_t g_tail;
static size_t g_txframes;
static size_t g_txwrites;
static size_t g_rxreads;
static unsigned int g_writes;
static unsigned int g_delays;
static unsigned int g_suspends;
static unsigned int g_chunks;
static uint32_t g_status;
static uint32_t g_divider;
static uint32_t g_cycles;
static uint32_t g_tick;
static bool g_clock_ready;
static bool g_extra_rx;
static bool g_stall;
static bool g_stall_after_first;
static bool g_suspend_stall;
static bool g_inject_modf;

static uint32_t getreg32(uintptr_t address)
{
  if (address == STM32_RCC_CCIPR9)
    {
      return RCC_CCIPR9_SPI5SEL_HSI_DIV_CK;
    }

  if (address == STM32_RCC_SR)
    {
      return g_clock_ready ? RCC_SR_HSIRDY : 0;
    }

  if (address == STM32_RCC_HSICFGR)
    {
      return g_divider << RCC_HSICFGR_HSIDIV_SHIFT;
    }

  if (address == DWT_CYCCNT)
    {
      g_cycles += g_tick;
      return g_cycles;
    }

  assert(address >= TEST_SPI_BASE && address <= TEST_SPI_BASE + 0x30);
  if (address == TEST_SPI_BASE + STM32_SPI_SR_OFFSET)
    {
      uint32_t status = g_status;

      if (g_inject_modf && g_txframes != 0)
        {
          g_inject_modf = false;
          g_status |= SPI_SR_MODF;
          g_regs[STM32_SPI_CR1_OFFSET / 4] &= ~SPI_CR1_SPE;
          g_regs[STM32_SPI_CFG2_OFFSET / 4] &= ~SPI_CFG2_MASTER;
          g_head = g_tail;
          return g_status;
        }

      if (!g_stall && !(g_stall_after_first && g_txframes != 0))
        {
          status |= SPI_SR_TXP;
        }

      if (g_head != g_tail)
        {
          status |= 1u << SPI_SR_RXPLVL_SHIFT;
        }

      if ((g_regs[STM32_SPI_CR1_OFFSET / 4] & SPI_CR1_CSTART) != 0 &&
          g_txframes == g_regs[STM32_SPI_CR2_OFFSET / 4])
        {
          g_status |= SPI_SR_EOT | SPI_SR_TXC;
          status |= SPI_SR_EOT | SPI_SR_TXC;
          g_regs[STM32_SPI_CR1_OFFSET / 4] &= ~SPI_CR1_CSTART;
        }

      return status;
    }

  return g_regs[(address - TEST_SPI_BASE) / 4];
}

static void putreg32(uint32_t value, uintptr_t address)
{
  unsigned int offset = address - TEST_SPI_BASE;

  assert(offset <= 0x30);
  g_writes++;
  if (offset == STM32_SPI_IFCR_OFFSET)
    {
      g_status &= ~value;
      return;
    }

  if (offset == STM32_SPI_CR2_OFFSET)
    {
      g_txframes = 0;
      g_chunks++;
    }

  if (offset == STM32_SPI_CR1_OFFSET && (value & SPI_CR1_CSUSP) != 0)
    {
      g_suspends++;
      if (!g_suspend_stall)
        {
          value &= ~SPI_CR1_CSTART;
          g_status |= SPI_SR_SUSP;
        }
      else
        {
          value |= SPI_CR1_CSTART;
        }
    }

  if (offset == STM32_SPI_CR1_OFFSET &&
      (g_regs[offset / 4] & SPI_CR1_SPE) != 0 &&
      (value & SPI_CR1_SPE) == 0)
    {
      assert(g_head == g_tail);
    }

  g_regs[offset / 4] = value;
}

static uint16_t read_frame(uintptr_t address, unsigned int nbits)
{
  assert(address == TEST_SPI_BASE + STM32_SPI_RXDR_OFFSET);
  assert((g_regs[STM32_SPI_CFG1_OFFSET / 4] & SPI_CFG1_DSIZE_MASK) ==
         nbits - 1);
  assert(g_head != g_tail);
  g_rxreads++;
  return g_fifo[g_head++ % 8];
}

static uint8_t getreg8(uintptr_t address)
{
  return (uint8_t)read_frame(address, 8);
}

static uint16_t getreg16(uintptr_t address)
{
  return read_frame(address, 16);
}

static void write_frame(uint16_t value, uintptr_t address,
                        unsigned int nbits)
{
  assert(address == TEST_SPI_BASE + STM32_SPI_TXDR_OFFSET);
  assert((g_regs[STM32_SPI_CFG1_OFFSET / 4] & SPI_CFG1_DSIZE_MASK) ==
         nbits - 1);
  assert(g_tail - g_head < 8);
  g_fifo[g_tail++ % 8] = value;
  g_txframes++;
  g_txwrites++;
  if (g_extra_rx && g_txframes == g_regs[STM32_SPI_CR2_OFFSET / 4])
    {
      g_fifo[g_tail++ % 8] = 0xaa;
    }
}

static void putreg8(uint8_t value, uintptr_t address)
{
  write_frame(value, address, 8);
}

static void putreg16(uint16_t value, uintptr_t address)
{
  write_frame(value, address, 16);
}

static void up_udelay(unsigned int delay)
{
  assert(delay == 1);
  g_delays++;
}

/* DRIVER_ROUTINES */

static struct stm32_spi_priv_s reset(void)
{
  struct stm32_spi_priv_s priv =
  {
    .base = TEST_SPI_BASE,
    .bus = 5,
    .nbits = 8,
    .mode = SPIDEV_MODE0,
    .state = SPI_STATE_READY,
    .frequency = STM32_HSI_FREQUENCY / 256,
    .kernel_frequency = STM32_HSI_FREQUENCY,
    .ccipr_mask = RCC_CCIPR9_SPI5SEL_MASK,
    .ccipr_source = RCC_CCIPR9_SPI5SEL_HSI_DIV_CK
  };

  memset(g_regs, 0, sizeof(g_regs));
  g_regs[STM32_SPI_CR1_OFFSET / 4] = SPI_CR1_SSI;
  g_regs[STM32_SPI_CFG1_OFFSET / 4] =
    SPI_CFG1_DSIZE_8BIT | SPI_CFG1_MBR_DIV256;
  g_regs[STM32_SPI_CFG2_OFFSET / 4] =
    SPI_CFG2_AFCNTR | SPI_CFG2_MASTER | SPI_CFG2_SSM;
  g_head = g_tail = g_txframes = g_txwrites = g_rxreads = 0;
  g_writes = g_delays = g_suspends = g_chunks = 0;
  g_status = g_divider = g_cycles = 0;
  g_tick = 1;
  g_clock_ready = true;
  g_extra_rx = g_stall = g_suspend_stall = g_inject_modf = false;
  g_stall_after_first = false;
  return priv;
}

static void test_configuration(void)
{
  struct stm32_spi_priv_s priv = reset();
  struct spi_dev_s *dev = &priv.dev;

  for (enum spi_mode_e mode = SPIDEV_MODE0; mode <= SPIDEV_MODE3; mode++)
    {
      spi_setmode(dev, mode);
      assert(priv.mode == mode && priv.last_error == OK);
      g_writes = g_delays = 0;
      spi_setmode(dev, mode);
      assert(g_writes == 0 && g_delays == 0);
    }

  spi_setmode(dev, 99);
  assert(priv.last_error == -EINVAL && priv.mode == SPIDEV_MODE3);
  for (int nbits = 8; nbits <= 16; nbits += 8)
    {
      spi_setbits(dev, nbits);
      assert(priv.nbits == nbits && priv.last_error == OK);
      g_writes = g_delays = 0;
      spi_setbits(dev, nbits);
      assert(g_writes == 0 && g_delays == 0);
    }

  spi_setbits(dev, 9);
  assert(priv.last_error == -EINVAL && priv.nbits == 16);
  g_regs[STM32_SPI_CR1_OFFSET / 4] |= SPI_CR1_SPE;
  spi_setmode(dev, priv.mode);
  assert(priv.last_error == -EBUSY);
  spi_setbits(dev, priv.nbits);
  assert(priv.last_error == -EBUSY);
  assert(spi_setfrequency(dev, priv.frequency) == 0);
  assert(priv.last_error == -EBUSY && g_writes == 0);

  g_regs[STM32_SPI_CR1_OFFSET / 4] &= ~SPI_CR1_SPE;
  g_regs[STM32_SPI_CFG2_OFFSET / 4] &= ~SPI_CFG2_MASTER;
  spi_setmode(dev, priv.mode);
  assert(g_writes != 0 && g_delays == 1);
  assert((g_regs[STM32_SPI_CFG2_OFFSET / 4] & SPI_CFG2_MASTER) != 0);

  g_writes = g_delays = 0;
  g_status = SPI_SR_OVR;
  spi_setbits(dev, priv.nbits);
  assert(g_writes != 0 && g_delays == 1);

  g_writes = g_delays = 0;
  priv.state = SPI_STATE_FAULTED;
  spi_setbits(dev, priv.nbits);
  assert(g_writes != 0 && priv.state == SPI_STATE_FAULTED);
}

static void test_frequency(void)
{
  static const unsigned int dividers[] = {2, 4, 8, 16, 32, 64, 128, 256};
  struct stm32_spi_priv_s priv = reset();
  struct spi_dev_s *dev = &priv.dev;

  for (g_divider = 0; g_divider < 4; g_divider++)
    {
      uint32_t kernel = STM32_HSI_FREQUENCY >> g_divider;

      for (unsigned int i = 0; i < 8; i++)
        {
          for (int delta = -1; delta <= 1; delta++)
            {
              uint32_t requested = kernel / dividers[i] + delta;
              uint32_t expected = 0;
              unsigned int index;

              for (index = 0; index < 8; index++)
                {
                  if (kernel / dividers[index] <= requested)
                    {
                      expected = kernel / dividers[index];
                      break;
                    }
                }

              assert(spi_setfrequency(dev, requested) == expected);
              assert(priv.last_error == (expected == 0 ? -ERANGE : OK));
              if (expected != 0)
                {
                  assert(priv.kernel_frequency == kernel);
                  assert(priv.frequency == expected);
                  assert((g_regs[STM32_SPI_CFG1_OFFSET / 4] &
                          SPI_CFG1_MBR_MASK) ==
                         index << SPI_CFG1_MBR_SHIFT);
                  g_writes = g_delays = 0;
                  assert(spi_setfrequency(dev, requested) == expected);
                  assert(g_writes == 0 && g_delays == 0);
                }
            }
        }
    }

  assert(spi_setfrequency(dev, 0) == 0 && priv.last_error == -EINVAL);
  g_clock_ready = false;
  assert(spi_setfrequency(dev, priv.frequency) == 0);
  assert(priv.last_error == -ENODEV && g_writes == 0);
}

static void test_transfers(void)
{
  struct stm32_spi_priv_s priv = reset();
  uint8_t *tx = malloc(2 * 65536 + 1);
  uint8_t *rx = malloc(2 * 65536 + 1);

  assert(tx != NULL && rx != NULL);
  for (size_t i = 0; i < 2 * 65536 + 1; i++)
    {
      tx[i] = (uint8_t)i;
    }

  for (unsigned int nbits = 8; nbits <= 16; nbits += 8)
    {
      size_t bytes = 65536 * (nbits / 8);

      spi_setbits(&priv.dev, nbits);
      g_txwrites = g_rxreads = g_chunks = 0;
      memset(rx, 0, 2 * 65536 + 1);
      assert(spi_transfer(&priv, tx + 1, rx + 1, 65536) == OK);
      assert(memcmp(tx + 1, rx + 1, bytes) == 0);
      assert(g_txwrites == 65536 && g_rxreads == 65536 && g_chunks == 2);
      assert(g_head == g_tail);
      assert((g_regs[STM32_SPI_CR1_OFFSET / 4] & SPI_CR1_SPE) == 0);
      assert(spi_transfer(&priv, tx + 1, NULL, 3) == OK);
      memset(rx, 0, 8);
      assert(spi_transfer(&priv, NULL, rx + 1, 3) == OK);
      for (size_t i = 1; i <= 3 * (nbits / 8); i++)
        {
          assert(rx[i] == 0xff);
        }
    }

  free(tx);
  free(rx);
}

static void test_errors(void)
{
  struct stm32_spi_priv_s priv = reset();
  uint8_t rx[3] = {0x55, 0x55, 0x55};
  uint8_t tx[] = {1, 2};

  g_extra_rx = true;
  assert(spi_transfer(&priv, tx, rx, 2) == -EIO);
  assert(rx[0] == 1 && rx[1] == 2 && rx[2] == 0x55);
  assert(g_head == g_tail && g_rxreads == 3);
  assert(priv.state == SPI_STATE_READY);

  priv = reset();
  g_stall = true;
  g_tick = 10000;
  assert(spi_transfer(&priv, tx, rx, 2) == -ETIMEDOUT);
  assert(g_txwrites == 0 && g_suspends == 0);
  assert(priv.state == SPI_STATE_READY);

  priv = reset();
  g_inject_modf = true;
  assert(spi_transfer(&priv, tx, rx, 2) == -EIO);
  assert(priv.state == SPI_STATE_READY);
  assert((g_regs[STM32_SPI_CFG2_OFFSET / 4] & SPI_CFG2_MASTER) != 0);
  assert((g_status & SPI_SR_MODF) == 0);

  priv = reset();
  g_stall_after_first = true;
  g_tick = 10000;
  assert(spi_transfer(&priv, tx, rx, 2) == -ETIMEDOUT);
  assert(g_txwrites == 1 && g_suspends == 1);
  assert(priv.state == SPI_STATE_READY);
  assert((g_regs[STM32_SPI_CR1_OFFSET / 4] & SPI_CR1_SPE) == 0);
  g_stall_after_first = false;
  assert(spi_transfer(&priv, tx, rx, 2) == OK);

  priv = reset();
  g_stall_after_first = true;
  g_suspend_stall = true;
  g_tick = 10000;
  assert(spi_transfer(&priv, tx, rx, 2) == -ETIMEDOUT);
  assert(g_suspends == 1 && priv.state == SPI_STATE_FAULTED);
  g_writes = 0;
  assert(spi_transfer(&priv, tx, rx, 2) == -EIO && g_writes == 0);

  priv = reset();
  g_divider = 1;
  assert(spi_transfer(&priv, tx, rx, 2) == -EIO && g_writes == 0);
}

int main(void)
{
  test_configuration();
  test_frequency();
  test_transfers();
  test_errors();
  puts("STM32N6 SPI driver tests passed");
  return EXIT_SUCCESS;
}
